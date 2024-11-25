import rclpy  # ROS 2 Python-Client-Bibliothek
from rclpy.node import Node  # Basisklasse für ROS 2 Nodes
from std_msgs.msg import Float32MultiArray  # Nachrichtentyp für das Publizieren von Arrays
import cv2  # OpenCV-Bibliothek für Bildverarbeitung
from sensor_msgs.msg import Image  # Nachrichtentyp für Bilder
from cv_bridge import CvBridge  # Brücke zwischen OpenCV und ROS

class CameraNode(Node):
    def __init__(self):
        # Initialisierung des Nodes mit dem Namen 'camera_node'
        super().__init__('camera_node')

        # Erstellen eines Publishers, der Float32MultiArray-Nachrichten auf dem Topic 'line_coordinates' veröffentlicht
        self.publisher_coordinates = self.create_publisher(Float32MultiArray, 'line_coordinates', 10)
        self.publisher_image = self.create_publisher(Image, 'camera_topic', 10)

        self.bridge = CvBridge()  # Initialisieren der CvBridge für den Bildtransfer

        # Öffnen der Webcam mit OpenCV (only LiNUX)
        self.cap = cv2.VideoCapture(2)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Überprüfen, ob die Kamera geöffnet werden konnte
        if not self.cap.isOpened():
            self.get_logger().error("Kamera konnte nicht geöffnet werden.")
            return
        
        # Timer, der die Funktion 'publish_coordinates and publish_frame' alle 0.1 Sekunden aufruft (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_coordinates)
        self.timer = self.create_timer(0.1, self.publish_frame)  

    def publish_frame(self):
        # Frame von der Webcam lesen
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Fehler beim Lesen des Frames.")
            return

        # OpenCV-Bild in ROS-Image-Nachricht konvertieren
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_image.publish(ros_image)


    def publish_coordinates(self):
        # coordinates von der Webcam lesen
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Fehler beim Lesen des coordinates.")
            return
    
        # Bestimmen der Höhe und Breite des Frames
        height, width = frame.shape[:2]

        # Definieren eines 100 Pixel hohen horizontalen Streifens in der Mitte des Bildes
        y_start = 400
        y_end = 450
        strip = frame[y_start:y_end, :]  # Extrahieren des Streifens

        # Umwandeln des Streifens in ein Graustufenbild
        gray = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY)

        # Schwellenwertanpassung zur Extraktion der schwarzen Linie
        _, thresholded = cv2.threshold(gray, 50, 100, cv2.THRESH_BINARY_INV)

        # Finden der Konturen im binären Bild
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Finden der größten Kontur basierend auf der Fläche
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)  # Berechnung der geometrischen Momente der größten Kontur

            if M["m00"] != 0:
                # Berechnung der Schwerpunktskoordinaten (cX und cY)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"]) + y_start  # Korrigierter Y-Wert, um im Kontext des Originalbilds zu sein

                # Erstellen und Publizieren der Nachricht mit den Koordinaten
                # Daten werden von Datatype float definiert
                msg = Float32MultiArray()
                msg.data = [float(cX), float(cY)]
                self.publisher_coordinates.publish(msg)
    
                # Ausgabe der Koordinaten zur Überprüfung im Log
                self.get_logger().info(f"cX: {cX}, cY: {cY}")

                # Zeichnen des Schwerpunkts als roten Punkt auf dem Streifen (nur zur Visualisierung)
                cv2.circle(strip, (cX, cY - y_start), 5, (0, 0, 255), -1) 

                # Zeichne die größte Kontur auf dem Streifen zur Visualisierung
                cv2.drawContours(strip, [largest_contour], -1, (0, 255, 0), 2)  # Zeichnet die Kontur in Grün

        # Setze den bearbeiteten Streifen wieder in das Originalbild ein
        frame[y_start:y_end, :] = strip

        # Anzeigen des Streifens und des Originalbildes in separaten Fenstern (optional für Debugging)
        cv2.imshow("Erkennung", strip)
        cv2.imshow("Threshold", thresholded)
        cv2.imshow("Original", frame)

        # Überprüfen, ob die Taste 'q' gedrückt wurde, um die Anzeige zu beenden
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    # Initialisierung der rclpy-Bibliothek
    rclpy.init(args=args)

    # Instanziierung und Start des Camera-Node
    node = CameraNode()
    rclpy.spin(node)  # Startet den Node und hält ihn aktiv, bis er manuell gestoppt wird

    # Freigeben der Ressourcen (Kamera und OpenCV-Fenster schließen)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()  # Beendet rclpy

if __name__ == '__main__':
    main()


