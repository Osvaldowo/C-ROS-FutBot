#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Guardamos la configuración original de la terminal
settings = termios.tcgetattr(sys.stdin)

class DualTeleop(Node):
    def __init__(self):
        super().__init__('dual_teleop')
        
        # Publicadores para ambos robots
        self.pub_robot1 = self.create_publisher(Twist, '/robot_1/cmd_vel', 10)
        self.pub_robot2 = self.create_publisher(Twist, '/robot_2/cmd_vel', 10)

        self.speed = 0.88
        self.turn = 2.0

        self.v1_x = 0.0
        self.v1_z = 0.0
        self.v2_x = 0.0
        self.v2_z = 0.0

        self.get_logger().info('¡Teleoperador Iniciado (Modo Terminal Seguro)!')
        self.get_logger().info('ROBOT 1: Teclas W A S D')
        self.get_logger().info('ROBOT 2: Teclas I J K L (I=Adelante, K=Atrás, J=Izq, L=Der)')
        self.get_logger().info('Manten presionada la tecla para mover, suelta para detener.')
        self.get_logger().info('Presiona CTRL+C para salir.')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # Escuchamos la terminal por 0.05 segundos
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        timeout1 = 0
        timeout2 = 0
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key:
                    # Controles Robot 1
                    if key.lower() in ['w', 'a', 's', 'd']:
                        timeout1 = 0
                        if key.lower() == 'w': self.v1_x = self.speed; self.v1_z = 0.0
                        elif key.lower() == 's': self.v1_x = -self.speed; self.v1_z = 0.0
                        elif key.lower() == 'a': self.v1_z = self.turn; self.v1_x = 0.0
                        elif key.lower() == 'd': self.v1_z = -self.turn; self.v1_x = 0.0
                        
                    # Controles Robot 2
                    elif key.lower() in ['i', 'j', 'k', 'l']:
                        timeout2 = 0
                        if key.lower() == 'i': self.v2_x = self.speed; self.v2_z = 0.0
                        elif key.lower() == 'k': self.v2_x = -self.speed; self.v2_z = 0.0
                        elif key.lower() == 'j': self.v2_z = self.turn; self.v2_x = 0.0
                        elif key.lower() == 'l': self.v2_z = -self.turn; self.v2_x = 0.0
                        
                    # Salir
                    elif key == '\x03': # CTRL+C
                        break
                else:
                    # Si no se presiona nada, los frenamos suavemente
                    timeout1 += 1
                    timeout2 += 1
                    if timeout1 > 3:
                        self.v1_x = 0.0; self.v1_z = 0.0
                    if timeout2 > 3:
                        self.v2_x = 0.0; self.v2_z = 0.0

                # Publicar velocidades
                twist1 = Twist()
                twist1.linear.x = self.v1_x
                twist1.angular.z = self.v1_z
                self.pub_robot1.publish(twist1)

                twist2 = Twist()
                twist2.linear.x = self.v2_x
                twist2.angular.z = self.v2_z
                self.pub_robot2.publish(twist2)

                rclpy.spin_once(self, timeout_sec=0.01)

        except Exception as e:
            print(e)
        finally:
            # Asegurar que se detengan al cerrar el script
            twist = Twist()
            self.pub_robot1.publish(twist)
            self.pub_robot2.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = DualTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()