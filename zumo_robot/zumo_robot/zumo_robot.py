import cv2
import numpy as np
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Serielle Verbindung mit Arduino herstellen (angepasster Port und Baudrate)
ser = serial.Serial('/dev/ttyACM0', 9600)  # Passe den Port je nach Betriebssystem an

# ROS 2 Node Klasse erstellen
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(String, 'camera_data', 10)
        self.cap = cv2.VideoCapture(2)

        if not self.cap.isOpened():
            print("Fehler: Kamera konnte nicht geöffnet werden.")
            exit()

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Fehler: Frame konnte nicht gelesen werden.")
            return

        height, width = frame.shape[:2]
        y_start = height // 2 - 50
        y_end = height // 2 + 50
        strip = frame[y_start:y_end, :]
        gray = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(gray, 50, 100, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                cY_global = cY + y_start

                # Sende die Koordinaten an Arduino
                data_to_send = f'{cX},{cY_global}\n'
                ser.write(data_to_send.encode('utf-8'))

                # Logge und veröffentliche die Koordinaten
                print(f"Schwerpunkt der größten Kontur: ({cX}, {cY_global})")
                msg = String()
                msg.data = data_to_send
                self.publisher_.publish(msg)

                cv2.circle(strip, (cX, cY), 5, (0, 0, 255), -1)
                cv2.drawContours(strip, [largest_contour], -1, (0, 255, 0), 2)

        frame[y_start:y_end, :] = strip
        cv2.imshow("Erkennung", strip)
        cv2.imshow("Threshold", thresholded)
        cv2.imshow("Original", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

# ROS 2 initialisieren und Node starten
def main():
    rclpy.init()
    camera_publisher = CameraPublisher()
    try:
        while rclpy.ok():
            camera_publisher.process_frame()
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

