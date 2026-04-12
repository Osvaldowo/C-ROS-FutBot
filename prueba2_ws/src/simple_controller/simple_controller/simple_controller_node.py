import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # <-- CAMBIO: Ahora usamos Float32 para coincidir con C++
from geometry_msgs.msg import Twist
import time
import math

from simple_controller.simplecontroller import Board, OUTPUT

# --- CONFIGURACIÓN FÍSICA DEL ROBOT (¡Ajusta estos valores!) ---
DIAMETRO_RUEDA_MM = 65.0  # Diámetro de tu llanta en milímetros
TICKS_POR_VUELTA = 341.2  # Ticks que da tu encoder por cada 1 vuelta completa de la llanta
CIRCUNFERENCIA_MM = math.pi * DIAMETRO_RUEDA_MM

# --- PINES DEL PUENTE H TB6612FNG (ESP32-S3) ---
PIN_PWMA = 4
PIN_AIN1 = 5
PIN_AIN2 = 6

PIN_PWMB = 7
PIN_BIN1 = 15
PIN_BIN2 = 16

PUERTO = "/dev/ttyACM0" 
BAUDIOS = 115200

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('simple_controller_node')
        self.get_logger().info(f"Conectando al ESP32 en {PUERTO}...")
        
        try:
            self.board = Board(PUERTO, BAUDIOS)
        except Exception as e:
            self.get_logger().error(f"Fallo al conectar: {e}")
            raise SystemExit

        pines_motor = [PIN_PWMA, PIN_AIN1, PIN_AIN2, PIN_PWMB, PIN_BIN1, PIN_BIN2]
        for pin in pines_motor:
            self.board.pinMode(pin, OUTPUT)

        self.current_dir_a = None
        self.current_pwm_a = -1.0
        self.current_dir_b = None
        self.current_pwm_b = -1.0
        self.detener_motores()

        self.last_ticks_a = 0
        self.last_ticks_b = 0
        self.last_time = time.time()
        self.primer_ciclo = True

        # --- PUBLICADORES (Tópicos Sincronizados con el nodo C++) ---
        self.pub_vel_izq = self.create_publisher(Float32, '/motor_vel_left', 10)
        self.pub_vel_der = self.create_publisher(Float32, '/motor_vel_right', 10)
        
        # Suscriptor
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer a 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Nodo ESP32 Sincronizado. Esperando comandos...")

    def leer_encoder_seguro(self, motor_id):
        puerto = self.board._Board__serialport
        try:
            puerto.reset_input_buffer()
        except Exception:
            pass
            
        puerto.write(bytearray([135, motor_id, 0, 0]))
        
        tiempo_inicio = time.time()
        while puerto.in_waiting < 4:
            if (time.time() - tiempo_inicio) > 0.02:
                return None  
            time.sleep(0.001) 
            
        while puerto.in_waiting > 0:
            byte = puerto.read(1)
            if byte and ord(byte) == 135:
                datos = puerto.read(3)
                if len(datos) == 3:
                    valor = (datos[0] << 16) | (datos[1] << 8) | datos[2]
                    if valor & 0x800000:
                        valor -= 0x1000000
                    return valor
        return None

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        MAX_LIN_VEL = 0.88   
        MAX_ANG_VEL = 10.6   
        
        # --- NUEVO: COMPENSACIÓN DE ZONA MUERTA ---
        # Este es el % mínimo de PWM necesario para que el motor apenas empiece a girar.
        # 0.25 = 25%. Si el robot aún se atasca, súbelo a 0.30. Si gira muy brusco al final, bájalo a 0.20.
        MIN_PWM = 0.25 

        # Calculamos la base de velocidad solicitada
        if abs(v) > 0.001:
            pwm_base = abs(v) / MAX_LIN_VEL
        elif abs(w) > 0.001:
            pwm_base = abs(w) / MAX_ANG_VEL
        else:
            pwm_base = 0.0

        # Mapeo: Si piden velocidad, empezamos desde el MIN_PWM hacia arriba
        if pwm_base > 0.0:
            pwm_final = MIN_PWM + (pwm_base * (1.0 - MIN_PWM))
        else:
            pwm_final = 0.0

        # Asegurar que nunca pase de 1.0 (100%)
        pwm_final = min(pwm_final, 1.0)
        # ------------------------------------------

        vel_izq = v - w
        vel_der = v + w

        # Extraemos las direcciones
        sig_izq = 1 if vel_izq > 0 else (-1 if vel_izq < 0 else 0)
        sig_der = 1 if vel_der > 0 else (-1 if vel_der < 0 else 0)

        # Aplicamos dirección * poder
        self.set_motor_izquierdo(sig_izq * pwm_final)
        self.set_motor_derecho(sig_der * pwm_final)

    def set_motor_izquierdo(self, velocidad):
        if velocidad > 0.05:
            dir_a = 1 
        elif velocidad < -0.05:
            dir_a = -1 
        else:
            dir_a = 0 

        pwm = min(abs(velocidad), 1.0)

        if dir_a != self.current_dir_a:
            if dir_a == 1:
                self.board.digitalWrite(PIN_AIN1, True)
                self.board.digitalWrite(PIN_AIN2, False)
            elif dir_a == -1:
                self.board.digitalWrite(PIN_AIN1, False)
                self.board.digitalWrite(PIN_AIN2, True)
            else:
                self.board.digitalWrite(PIN_AIN1, False)
                self.board.digitalWrite(PIN_AIN2, False)
            self.current_dir_a = dir_a

        if pwm != self.current_pwm_a:
            self.board.analogWrite(PIN_PWMA, pwm)
            self.current_pwm_a = pwm

    def set_motor_derecho(self, velocidad):
        if velocidad > 0.05:
            dir_b = 1
        elif velocidad < -0.05:
            dir_b = -1
        else:
            dir_b = 0

        pwm = min(abs(velocidad), 1.0)

        if dir_b != self.current_dir_b:
            if dir_b == 1:
                self.board.digitalWrite(PIN_BIN1, True)
                self.board.digitalWrite(PIN_BIN2, False)
            elif dir_b == -1:
                self.board.digitalWrite(PIN_BIN1, False)
                self.board.digitalWrite(PIN_BIN2, True)
            else:
                self.board.digitalWrite(PIN_BIN1, False)
                self.board.digitalWrite(PIN_BIN2, False)
            self.current_dir_b = dir_b

        if pwm != self.current_pwm_b:
            self.board.analogWrite(PIN_PWMB, pwm)
            self.current_pwm_b = pwm

    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.last_time

        ticks_a = self.leer_encoder_seguro(0)
        time.sleep(0.002) 
        ticks_b = self.leer_encoder_seguro(1)

        if ticks_a is not None:
            current_ticks_a = ticks_a
        else:
            current_ticks_a = self.last_ticks_a

        if ticks_b is not None:
            current_ticks_b = ticks_b
        else:
            current_ticks_b = self.last_ticks_b

        if self.primer_ciclo:
            self.last_ticks_a = current_ticks_a
            self.last_ticks_b = current_ticks_b
            self.last_time = current_time
            self.primer_ciclo = False
            return  # Ignoramos este ciclo para no calcular velocidades falsas

        if dt > 0:
            # 1. Calcular Ticks por segundo
            ticks_por_seg_a = (current_ticks_a - self.last_ticks_a) / dt
            ticks_por_seg_b = (current_ticks_b - self.last_ticks_b) / dt

            # 2. Convertir a mm/s
            vel_a_mms = (ticks_por_seg_a / TICKS_POR_VUELTA) * CIRCUNFERENCIA_MM
            vel_b_mms = (ticks_por_seg_b / TICKS_POR_VUELTA) * CIRCUNFERENCIA_MM

            # 3. Publicar como Float32
            msg_izq = Float32()
            msg_izq.data = float(vel_a_mms)
            self.pub_vel_izq.publish(msg_izq)

            msg_der = Float32()
            msg_der.data = float(vel_b_mms)
            self.pub_vel_der.publish(msg_der)

        self.last_ticks_a = current_ticks_a
        self.last_ticks_b = current_ticks_b
        self.last_time = current_time

    def detener_motores(self):
        self.board.digitalWrite(PIN_AIN1, False)
        self.board.digitalWrite(PIN_AIN2, False)
        self.board.digitalWrite(PIN_BIN1, False)
        self.board.digitalWrite(PIN_BIN2, False)
        self.board.analogWrite(PIN_PWMA, 0.0)
        self.board.analogWrite(PIN_PWMB, 0.0)
        self.current_dir_a = 0
        self.current_dir_b = 0
        self.current_pwm_a = 0.0
        self.current_pwm_b = 0.0

def main(args=None):
    rclpy.init(args=args)
    nodo_motores = MotorControllerNode()

    try:
        rclpy.spin(nodo_motores)
    except KeyboardInterrupt:
        pass
    finally:
        nodo_motores.detener_motores()
        if hasattr(nodo_motores, 'board'):
            nodo_motores.board.close()
        nodo_motores.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()