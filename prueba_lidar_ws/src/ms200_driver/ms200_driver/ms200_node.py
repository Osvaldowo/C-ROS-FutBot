#!/usr/bin/env python3
"""
Driver ROS 2 para el LiDAR MS200 / LD06.

Este nodo se conecta mediante puerto serial al LiDAR, decodifica el protocolo
de comunicación de 47 bytes, extrae los puntos de escaneo y publica un 
mensaje `sensor_msgs/msg/LaserScan`. Incluye un filtro de suavizado espacial 
y temporal (EMA) para reducir el ruido y la oscilación en las lecturas.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math

# CONFIGURACIÓN DEL PROTOCOLO MS200 / LD06
HEADER = b'\x54\x2C'
PACKET_SIZE = 47

class MS200Node(Node):
    """
    Nodo de ROS 2 para procesar y publicar datos del LiDAR MS200.

    Parámetros de ROS:
        port (str): Puerto serial donde está conectado el LiDAR (default: '/dev/ttyUSB0').
        baudrate (int): Tasa de baudios para la comunicación serial (default: 230400).
        frame_id (str): Marco de referencia para el mensaje LaserScan (default: 'laser_frame').

    Publicadores:
        /scan (sensor_msgs/msg/LaserScan): Tópico donde se publican los datos del LiDAR.
    """

    def __init__(self):
        super().__init__('ms200_node')
        
        # Declaración e inicialización de parámetros
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publicador del escaneo
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Conexión serial
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Conectado al LiDAR en {port} a {baud}')
        except Exception as e:
            self.get_logger().error(f'No se pudo conectar al puerto {port}: {e}')
            return

        # Buffers de datos
        self.scan_points = []  
        self.current_packet = bytearray()
        
        # --- CEREBRO ANTI-OSCILACIÓN ---
        # Memoria estática para los 360 grados, inicializada al rango máximo (12.0m)
        self.distancias_suavizadas = [12.0] * 360 
        
        # Factor Alpha para el filtro EMA (Exponential Moving Average).
        # Define el peso de la lectura actual vs el historial.
        # 0.25 significa: 25% lectura nueva, 75% historial.
        self.alpha = 0.25 

        # Timer de alta frecuencia para lectura del puerto serial
        self.create_timer(0.001, self.read_serial_data)

    def read_serial_data(self):
        """
        Lee los datos crudos del puerto serial y extrae paquetes válidos
        basados en el encabezado y el tamaño del paquete definido.
        """
        if not self.ser.is_open:
            return
            
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.current_packet.extend(data)
        except Exception as e:
            self.get_logger().error(f'Error leyendo serial: {e}')
            
        # Extraer y procesar paquetes mientras el buffer sea suficientemente grande
        while len(self.current_packet) >= PACKET_SIZE:
            if self.current_packet[0] == 0x54 and self.current_packet[1] == 0x2C:
                packet = self.current_packet[:PACKET_SIZE]
                self.parse_packet(packet)
                # Remover el paquete procesado del buffer
                self.current_packet = self.current_packet[PACKET_SIZE:]
            else:
                # Descartar un byte si el encabezado no coincide
                self.current_packet.pop(0)

    def parse_packet(self, data):
        """
        Decodifica un paquete de 47 bytes del MS200/LD06.
        Extrae los 12 puntos de medición (distancia y ángulo) y los almacena en el buffer.

        Args:
            data (bytearray): Paquete de datos crudos de longitud PACKET_SIZE.
        """
        try:
            speed = struct.unpack('<H', data[2:4])[0]
            start_angle = struct.unpack('<H', data[4:6])[0] / 100.0
            end_angle = struct.unpack('<H', data[42:44])[0] / 100.0
            
            # Corrección cuando el escáner cruza los 360 grados
            if end_angle < start_angle:
                end_angle += 360.0
                
            step = (end_angle - start_angle) / 11.0
            
            # Desempaquetar las 12 lecturas del paquete
            for i in range(12):
                base = 6 + (i * 3)
                distance_mm = struct.unpack('<H', data[base:base+2])[0]
                intensity = data[base+2] # La intensidad está disponible pero no se usa en este nodo
                
                angle = start_angle + (step * i)
                if angle >= 360.0:
                    angle -= 360.0
                
                # Filtrar valores fuera del rango útil (0 a 12 metros)
                if 0 < distance_mm <= 12000:
                    self.scan_points.append((angle, distance_mm / 1000.0))
            
            # Publicar el escaneo cuando se completa una vuelta (ángulo vuelve a iniciar)
            if start_angle < 10.0 and len(self.scan_points) > 100:
                self.publish_scan()
                
        except Exception as e:
            self.get_logger().warn(f'Error parseando paquete: {e}')

    def publish_scan(self):
        """
        Procesa el buffer de puntos crudos, aplica discretización espacial, 
        rellenado de huecos, y filtrado temporal (EMA), para luego publicar 
        el mensaje ROS `LaserScan` finalizado.
        """
        if not self.scan_points:
            return

        # 1. Crear un arreglo en blanco de exactamente 360 grados (infinito por defecto)
        scan_360 = [float('inf')] * 360
        
        # 2. Discretización espacial: Mapear puntos crudos a grados enteros (0 a 359)
        for angle, dist in self.scan_points:
            idx = int(angle) % 360
            # Retener solo el obstáculo más cercano por cada grado
            if dist < scan_360[idx]:
                scan_360[idx] = dist

        # 3. Rellenado de huecos y filtrado temporal (Filtro EMA)
        for i in range(360):
            # Interpolación simple de vecinos si falta lectura en un grado
            if scan_360[i] == float('inf'):
                scan_360[i] = scan_360[i-1] if i > 0 else 12.0
            
            # Aplicación de la Media Móvil Exponencial (EMA)
            self.distancias_suavizadas[i] = (self.alpha * scan_360[i]) + ((1.0 - self.alpha) * self.distancias_suavizadas[i])

        # 4. Construcción del mensaje ROS
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.range_min = 0.02  
        msg.range_max = 12.0  
        
        # Incremento angular estricto de 1 grado
        msg.angle_increment = (2.0 * math.pi) / 360.0 
        msg.time_increment = 0.0 
        
        # Asignar la memoria suavizada como el resultado final
        msg.ranges = self.distancias_suavizadas 
            
        self.publisher_.publish(msg)
        
        # Limpiar el buffer de puntos para la siguiente rotación
        self.scan_points = []

def main(args=None):
    rclpy.init(args=args)
    node = MS200Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
