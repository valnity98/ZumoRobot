import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        #serielle Schnittstelle öffnen (passe den Port ggf. an)
        self.serial_port = serial.Serial('/dev/ttyUSB0',9600,timeout=1)
        self.cmd_subscriber= self.create_subscription(String,'led_cmd',self.send_to_arduino,10)
      


    def send_to_arduino(self,msg):
        #send den LED-Befehl an Arduino
        self.serial_port.write(msg.data.encode())
        self.get_logger().info(f'sent command: {msg.data} to Arduino')



def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    