#!/usr/bin/env python3

"""
Camera Node for ROS 2
Author: Mutasem Bader, Felix Biermann
Description:
    - Captures frames from a camera using OpenCV.
    - Publishes detected line coordinates (x, y) as Float32MultiArray on the 'line_coordinates' topic.
    - Publishes the camera frames as ROS 2 Image messages on the 'camera_topic' topic.
    - Kalmanfilter

Requirements:
    - ROS 2 installation
    - OpenCV and cv_bridge libraries
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.interpolate import splprep, splev
from zumo_robot.log_node import LogPublisher


class CameraNode(Node):
    """ROS 2 Node for capturing frames and detecting line coordinates."""

    def __init__(self,log_publisher):
        super().__init__('camera_node')

        self.log_publisher = log_publisher
        # ROS 2 Publishers
        self.publisher_coordinates = self.create_publisher(Float32MultiArray, 'line_coordinates', 10)
        self.publisher_image = self.create_publisher(Image, 'camera_topic', 10)


        # OpenCV-ROS Bridge
        self.bridge = CvBridge()

        # Initialisierung des Kalman-Filters
        self.kalman = cv2.KalmanFilter(2, 1)  # 2 Zustände (Position, Geschwindigkeit), 1 Messung (Position)
        self.kalman.measurementMatrix = np.array([[1, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 1], [0, 1]], np.float32)
        self.kalman.processNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 0.03

        # Webcam öffnen
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        log_publisher.log("Camera Node ist on.")


        if not self.cap.isOpened():
            self.log_publisher.log("Failed to open camera. Please check the connection.", level = "error")
            rclpy.shutdown()
            return
        
        # Timer for periodic frame processing and publishing
        self.create_timer(0.1, self.process_frame)

    def process_frame(self):

        """Processes the camera frame and publishes line coordinates and frames."""
        ret, frame = self.cap.read()
        if not ret:
            self.log_publisher.log("Failed to capture frame.", level= "warn")
            return
        
        try:
        
            # In Graustufen umwandeln
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Einen binären Schwellenwert anwenden, um die schwarzen Bereiche hervorzuheben
            _, edges = cv2.threshold(gray, 60, 200, cv2.THRESH_BINARY_INV)
        
            # Höhe und Breite des Bildes ermitteln
            height, width = edges.shape
            
            # Das Bild in 5 horizontale Sektionen unterteilen
            sections = []
            section_height = height // 5
            
            for i in range(5):
                y_start = i * section_height
                y_end = (i + 1) * section_height if i < 4 else height  # Für die letzte Sektion bis zum Bildende
                section = edges[y_start:y_end, :]  # Abschnitt extrahieren
                sections.append((section, y_start, y_end))  # Sektion speichern mit den y-Koordinaten
        
            # Liste, um die Schwerpunkte zu speichern
            centroids = []
        
            # Konturen erkennen und die größte Kontur in jeder Sektion finden
            for section, y_start, y_end in sections:
                contours, _ = cv2.findContours(section, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
                # Falls Konturen gefunden wurden
                if contours:
                    # Filtere kleine Konturen (z.B. nur Konturen mit einer Mindestfläche)
                    min_contour_area = 100  # Mindestfläche der Konturen
                    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
                    
                    if contours:
                        # Die größte Kontur finden (nach Fläche)
                        largest_contour = max(contours, key=cv2.contourArea)
                        
                        # Die größte Kontur auf dem Originalbild zeichnen
                        # Wir verschieben die Konturen zurück in die richtige Position auf dem Gesamtbild
                        largest_contour_shifted = np.array([cnt + (0, y_start) for cnt in largest_contour])
                        cv2.drawContours(frame, [largest_contour_shifted], -1, (0, 255, 0), 3)
            
                        # Schwerpunkte der Kontur berechnen
                        M = cv2.moments(largest_contour_shifted)
                        if M['m00'] != 0:  # Verhindern einer Division durch null
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])

                            # Speichern des Schwerpunkts
                            centroids.append((cx, cy))
                            # Schwerpunkt als roten Punkt anzeigen
                            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)  # Roter Punkt für Schwerpunkt
        

            # Wenn Schwerpunkte gefunden wurden, berechnen wir den Durchschnittspunkt
            if centroids:
                #avg_cx = int(np.mean([c[0] for c in centroids]))  # Durchschnitt der x-Koordinaten
                avg_cy = int(np.mean([c[1] for c in centroids]))  # Durchschnitt der y-Koordinaten
                avg_cx = self.weighted_average_by_position(centroids)

                self.prediction = self.kalman.predict()

                # Korrektur basierend auf der aktuellen Messung
                self.kalman.correct(np.array([[np.float32(avg_cx)]]))
                
                 # Publish line coordinates
                msg = Float32MultiArray(data=[float(avg_cx), float(avg_cy)])
                self.publisher_coordinates.publish(msg)
                self.get_logger().info(f"Published coordinates: cx={cx}, cx={cy}")

                cv2.circle(frame, (avg_cx, avg_cy), 10, (255, 0, 0), -1)  # Blauer Punkt für Durchschnitt

                corrected_position = int(self.kalman.statePost[0][0])
                # Durchschnittspunkt als blauen Punkt anzeigen
                cv2.circle(frame, (corrected_position, avg_cy), 10, (0, 255, 0), -1)  
            
            # Publish the processed frame
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_image.publish(ros_image)

        
        except Exception as e:
            self.log_publisher.log(f"Error during frame processing: {e}", level= "error")

    def destroy_node(self):
        """Release camera resources on shutdown."""
        self.cap.release()
        super().destroy_node()

    
    def weighted_average_by_position(self, centroids, power=2):
        """
        Berechnet einen gewichteten Durchschnitt von x-Koordinaten der Zentroiden.
        Niedrigere Positionen im Array werden stärker gewichtet.

        :param centroids: Liste von Zentroiden [(x1, y1), (x2, y2), ...].
        :param power: Exponent für die Gewichtung, standardmäßig 2.
        :return: Gewichteter Durchschnitt der x-Koordinaten.
        """
        x_values = [c[0] for c in centroids]
        positions = np.arange(1, len(x_values) + 1)  # Positionen beginnen bei 1
        weights = (1 / positions) ** power  # Gewichtung umkehren: kleinere Positionen größere Gewichte
        weighted_avg = np.sum(np.array(x_values) * weights) / np.sum(weights)
        return int(weighted_avg)


def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    log_publisher = LogPublisher()
    node = CameraNode(log_publisher)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        log_publisher.log("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
