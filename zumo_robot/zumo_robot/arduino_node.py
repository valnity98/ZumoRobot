import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')

        # Publisher für Encoder-Daten
        self.publisher_ = self.create_publisher(Int16MultiArray, 'encoder_data', 10)

        # Serielle Verbindung zum Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Passen Sie den seriellen Port an
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            rclpy.shutdown()

        # Timer zum periodischen Abrufen der Encoder-Daten
        self.create_timer(0.1, self.read_encoder_data)

    def read_encoder_data(self):
        if self.serial_port.in_waiting >= 5:  # Stelle sicher, dass mindestens 5 Bytes verfügbar sind
            try:
                # Lese das 6-Byte-Paket von Arduino
                data = self.serial_port.read(6)
                if len(data) == 6 and data[0] == 0x02 and data[5] == 0x03:  # Prüfen auf STX und ETX
                    # Extrahieren der Encoder-Daten
                    left_encoder = int.from_bytes(data[1:3], byteorder='little', signed=True)
                    right_encoder = int.from_bytes(data[3:5], byteorder='little', signed=True)

                    # Erstellen und Veröffentlichen der Nachricht
                    msg = Int16MultiArray()
                    msg.data = [left_encoder, right_encoder]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Left: {left_encoder}, Right: {right_encoder}")
                else:
                    self.get_logger().warn("Received invalid data or incomplete packet.")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {str(e)}")

    def close(self):
        self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    rclpy.spin(node)
    node.close()

if __name__ == '__main__':
    main()
