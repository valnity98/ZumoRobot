import rclpy  # ROS 2 Python-Client-Bibliothek
from rclpy.node import Node  # Basisklasse für ROS 2 Nodes
from std_msgs.msg import Float32MultiArray  # Nachrichtentyp für das Publizieren von Arrays
import cv2  # OpenCV-Bibliothek für Bildverarbeitung

class CameraNode(Node):
    def __init__(self):
        # Initialisierung des Nodes mit dem Namen 'camera_node'
        super().__init__('camera_node')

        # Erstellen eines Publishers, der Float32MultiArray-Nachrichten auf dem Topic 'line_coordinates' veröffentlicht
        self.publisher_ = self.create_publisher(Float32MultiArray, 'line_coordinates', 10)

        # Öffnen der Webcam mit OpenCV (only LiNUX)
        self.cap = cv2.VideoCapture(2)

        # Überprüfen, ob die Kamera geöffnet werden konnte
        if not self.cap.isOpened():
            self.get_logger().error("Kamera konnte nicht geöffnet werden.")
            return
        
        # Timer, der die Funktion 'publish_coordinates' alle 0.1 Sekunden aufruft (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_coordinates)

    def publish_coordinates(self):
        # Frame von der Webcam lesen
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Fehler beim Lesen des Frames.")
            return
    
        # Bestimmen der Höhe und Breite des Frames
        height, width = frame.shape[:2]

        # Definieren eines 100 Pixel hohen horizontalen Streifens in der Mitte des Bildes
        y_start = height // 2 - 50
        y_end = height // 2 + 50
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
                msg = Float32MultiArray()
                msg.data = [float(cX), float(cY)]
                self.publisher_.publish(msg)
    
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


