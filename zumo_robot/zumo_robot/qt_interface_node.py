##################################################
#
# Topic: Qt Application to read data from encoder, motoren and video streaming
# Author: Mutasem Bader
# Relaese: Date: 17.11.2024
# Change: xxxxxx
#
##################################################


# Importiere notwendige Bibliotheken und Module für ROS2, OpenCV und die Qt-Oberfläche
import sys  # Systemfunktionen wie Kommandozeilenargumente
import os  # Betriebssystemfunktionen wie Pfadverwaltung
import rclpy  # ROS2 Python-Client-Bibliothek
from rclpy.node import Node  # Zum Erstellen eines ROS2-Knotens
from std_msgs.msg import Bool  # ROS2 Bool-Nachricht
from sensor_msgs.msg import Image  # ROS2 Image-Nachricht für Kamera-Daten
from cv_bridge import CvBridge  # Konvertierung zwischen ROS2-Bildnachrichten und OpenCV-Bildern
import cv2  # OpenCV-Bibliothek für Bildverarbeitung
from PyQt5.QtWidgets import QApplication, QMainWindow  # GUI-Komponenten
from PyQt5.QtGui import QImage, QPixmap  # Bildanzeige-Klassen
from PyQt5 import uic, QtCore  # UI-Dateien laden und QtCore für Threading

# Thread-Klasse zum Ausführen des ROS2-Knotens im Hintergrund, um die GUI nicht zu blockieren.
class ROS2Thread(QtCore.QThread):
    
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)  # Startet den ROS2-Node zur Verarbeitung von Nachrichten

# Hauptklasse der Qt-Anwendung
class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        
        # Lade die UI-Datei
        ui_path = os.path.join(os.path.dirname(__file__), os.getcwd() + '/src/zumo_robot/Qt/zumorobot.ui')
        uic.loadUi(ui_path, self)
        
        # Initialisiere den ROS2-Node
        rclpy.init()
        self.node = Node("qt_ros_interface")
        self.publisher = self.node.create_publisher(Bool, 'robot_command', 10)
        self.video_subscriber = self.node.create_subscription(Image, 'camera_topic', self.display_video, 10)
        
        self.bridge = CvBridge()  # Initialisiere CvBridge

        # Verknüpfe GUI-Buttons mit Funktionen
        self.Start_PushButton.clicked.connect(self.send_start_command)
        self.Stop_PushButton.clicked.connect(self.send_stop_command)

        # Starte den ROS2-Thread
        self.ros_thread = ROS2Thread(self.node)
        self.ros_thread.start()

    # Sendet den Start-Befehl an das Robotersystem.
    def send_start_command(self):
        msg = Bool(data=True)
        self.publisher.publish(msg)
        self.Status_LineEdit.setText("Robot started")
        
    # Sendet den Stop-Befehl an das Robotersystem.
    def send_stop_command(self):
        msg = Bool(data=False)
        self.publisher.publish(msg)
        self.Status_LineEdit.setText("Robot stopped")

    # Zeigt das Video von der Kamera in der GUI an.
    def display_video(self, msg):
  
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
        self.Video_Label.setPixmap(QPixmap.fromImage(q_img))
        
    # Räumt Ressourcen auf, wenn das Fenster geschlossen wird.
    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

# Hauptfunktion zur Ausführung der Qt-Anwendung
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

# Einstiegspunkt des Skripts
if __name__ == "__main__":
    main()
