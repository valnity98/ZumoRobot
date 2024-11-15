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
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Passen Sie den seriellen Port an

        # Timer zum periodischen Abrufen der Encoder-Daten
        self.create_timer(0.1, self.read_encoder_data)

    
    def read_encoder_data(self):
        if self.serial_port.in_waiting > 0:
            # Lesen des 7-Byte-Pakets von Arduino
            data = self.serial_port.read(7)

            if len(data) == 7 and data[0] == 0x02 and data[6] == 0x03:  # Prüfen auf STX und ETX
                # Berechne die Prüfziffer
                checksum = self.calculate_checksum(data)

                # Überprüfe die empfangene Prüfziffer
                if checksum == data[5]:
                    # Extrahieren der Encoder-Daten
                    left_encoder = int.from_bytes(data[1:3], byteorder='little', signed=True)
                    right_encoder = int.from_bytes(data[3:5], byteorder='little', signed=True)

                    # Erstellen und Veröffentlichen der Nachricht
                    msg = Int16MultiArray()
                    msg.data = [left_encoder, right_encoder]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Left: {left_encoder}, Right: {right_encoder}")
                else:
                    self.get_logger().warn("Checksumme ungültig! Daten werden ignoriert.")

    def calculate_checksum(self, data):
        """
        Berechnet die Prüfziffer für das übergebene Datenpaket.
        Die Prüfziffer ist das XOR der Datenbytes.
        """
        checksum = data[1] ^ data[2] ^ data[3] ^ data[4]
        return checksum

    def close(self):
        self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    rclpy.spin(node)
    node.close()

if __name__ == '__main__':
    main()
