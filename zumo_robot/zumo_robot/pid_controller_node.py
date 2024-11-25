# Importiere die notwendigen Module und Bibliotheken
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, Int16MultiArray
import serial
from Zumo_Library.PIDController import Zumo328PPID  # Import der PID-Controller-Logik
import struct
import numpy as np

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # Erstelle ROS2-Abonnements
        self.create_subscription(Float32MultiArray, 'line_coordinates', self.line_coordinates_callback, 10)
        self.create_subscription(Bool, 'robot_command', self.command_callback, 10)

        # Statusindikator für Aktivierung des Controllers
        self.is_active = False
        self.serial_port = None
        

        # Serielle Verbindung zum Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Passen Sie den seriellen Port an
        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f"Failed to open serial port: {str(e)}")

        # Initialisiere den PID-Controller
        self.pid_controller = Zumo328PPID(max_speed=150.0)

        # Publisher für Motorsteuerbefehle
        self.publisher_Motoren = self.create_publisher(Int16MultiArray, 'motor_speeds', 10)

    # Callback-Funktion, um die Koordinaten der Linie zu empfangen und die PID-Steuerung zu berechnen.
    def line_coordinates_callback(self, msg):

        #if not self.is_active:
        #   self.get_logger().info("PID Controller is inactive. Ignoring line coordinates.")
        #   return

        #self.get_logger().info(f"Received line coordinates: {msg.data}")
        cX, cY = msg.data  # Extrahiere die Koordinaten


        # control_speed(self, measured_position, target_position, kp, kd, aktiv=False):
        self.pid_controller.control_speed(cX,334, 0.6, 0.3 ,False)  # PID-Berechnung
        
        # Hole die berechneten Motorsteuerungen
        left_speed = np.int16(self.pid_controller.get_left_speed())
        right_speed = np.int16(self.pid_controller.get_right_speed())

        # Veröffentliche Motorsteuerbefehle, abhängig von der Controller-Aktivität
        motor_msg = Int16MultiArray()
        #motor_msg.data = [left_speed, right_speed] if self.is_active else [0, 0] 
        motor_msg.data = [int(left_speed), int(right_speed)]  
        self.publisher_Motoren.publish(motor_msg)

        # Sende Steuerbefehle an Arduino
        self.send_to_arduino(left_speed, right_speed)




    # Sendet die Motorsteuerbefehle über die serielle Verbindung an den Arduino.
    def send_to_arduino(self, left_speed, right_speed): 

        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not available. Skipping send.")
            return

        try:
            # Verpacke die Geschwindigkeitswerte als zwei 16-Bit-Werte
            data_to_send = struct.pack('<hh', left_speed, right_speed)
        
            # Füge Start- und End-Bytes hinzu
            start_byte = b'\x02'  # Start-Byte (STX)
            end_byte = b'\x03'    # End-Byte (ETX)
            # Kombiniere die Daten mit Start- und End-Bytes
            full_data = start_byte + data_to_send + end_byte
    
            # Sende die Daten über die serielle Verbindung
            self.serial_port.write(full_data)

            self.get_logger().info(f"Sent to Arduino: Left={left_speed}, Right={right_speed}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send data: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in send_to_arduino: {str(e)}")

       

    # Schließe die serielle Verbindung beim Beenden des Nodes
    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Closed serial port.")

    # Aktiviert oder deaktiviert den PID-Controller basierend auf eingehenden Befehlen.
    def command_callback(self, msg: Bool):
        self.is_active = msg.data
        if not self.is_active:
            # Motoren auf Null setzen, wenn der PID-Controller deaktiviert ist
            self.send_to_arduino(0, 0)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
