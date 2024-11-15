import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
from Zumo_Library import PIDController  # PID-Controller-Logik importieren

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.create_subscription(Float32MultiArray, 'line_coordinates', self.line_coordinates_callback, 10)
        # Serielle Verbindung zu Arduino
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Passen Sie den Port an "ttyUSB0"
        time.sleep(2)

        # Initialisiere den PID-Controller
        self.pid_controller = PIDController(max_speed=200.0)  # Beispiel: maxSpeed = 200.0

        # Publisher für Motorsteuerbefehle
        self.publisher_ = self.create_publisher(Float32MultiArray, 'motor_speeds', 10)

    def line_coordinates_callback(self, msg):
        cX, cY = msg.data
        # control = self.pid(cX)  # Berechnung der Fehlerkorrektur mit PID

        self.pid_controller.control_speed(cX, 400, 1, 1, False)

        # Hole die berechneten Motorsteuerbefehle
        left_speed = self.pid_controller.get_left_speed()
        right_speed = self.pid_controller.get_right_speed()

        motor_msg = Float32MultiArray()
        motor_msg.data = [left_speed, right_speed]
        self.publisher_.publish(motor_msg)  # Motorsteuerbefehle veröffentlichen

        # Steuerbefehle an Arduino senden
        self.send_to_arduino(left_speed, right_speed)

    def send_to_arduino(self, left_speed, right_speed):
        data_to_send = bytearray([0x02, left_speed & 0xFF, (left_speed >> 8) & 0xFF,
                                  right_speed & 0xFF, (right_speed >> 8) & 0xFF, 
                                  (left_speed ^ right_speed) & 0xFF, 0x03])
        self.serial_port.write(data_to_send)

    def close(self):
        self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.close()

if __name__ == '__main__':
    main()

