##################################################
#
# Topic: Erste Applikation für serial interface zwischen Ros2 und Ardruino mit der Library (pyserial) 
# Author: Mutasem Bader
# Relaese: Date: 04.11.2024
# Change: xxxxxx
#
##################################################


import rclpy
from rclpy.node import Node
import serial

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        #serielle Schnittstelle öffnen (passe den Port ggf. /ttyUSB0  an)
        self.serial_port = serial.Serial('/dev/ttyACM0',9600,timeout=1)
        #self.cmd_subscriber= self.create_subscription(String,'led_cmd',self.send_to_arduino,10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period,self.send_to_arduino)


    def send_to_arduino(self):
        #send den LED-Befehl an Arduino
        msg = '1'
        self.serial_port.write(msg.encode())
        self.get_logger().info(f'sent command: {msg} to Arduino')



def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    