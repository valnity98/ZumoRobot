import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')

        # Publisher für Encoder-Daten
        self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_data', 10)
        self.serial_port = None

        # Serielle Verbindung zum Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Passen Sie den seriellen Port an
        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f"Failed to open serial port: {str(e)}")

        # Timer zum periodischen Abrufen der Encoder-Daten
        self.create_timer(0.1, self.read_encoder_data)

    def read_encoder_data(self):
        if self.serial_port.in_waiting >= 10:  # Stelle sicher, dass mindestens 10 Bytes verfügbar sind
             
            try:
                # Lese das 10-Byte-Paket von Arduino
                data = self.serial_port.read(10)
                if len(data) == 10 and data[0] == 0x02 and data[9] == 0x03:  # Prüfen auf STX und ETX
                    # Extrahieren der Encoder-Daten
                    left_encoder = int.from_bytes(data[1:5], byteorder='little', signed=True)
                    right_encoder = int.from_bytes(data[5:9], byteorder='little', signed=True)

                    # Erstellen und Veröffentlichen der Nachricht
                    msg = Int32MultiArray()
                    msg.data = [left_encoder, right_encoder]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Left: {left_encoder}, Right: {right_encoder}")
                else:
                    self.get_logger().warn("Received invalid data or incomplete packet.")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {str(e)}")

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Closed serial port.")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.close()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
