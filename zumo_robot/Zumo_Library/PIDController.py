#include Dateien
import time

#Class "Zumo329PPID" für Steuerung der Motoren der Roboter
class Zumo328PPID:
    def __init__(self, max_speed=200.0):
        self.max_speed = max_speed
        self.left_speed = 0
        self.right_speed = 0
        self.prevT = 0
        self.last_error = 0

    def control_speed(self, measured_position, target_position, kp, kd, aktiv=False):
        # Der "Fehler" ist die Entfernung von der Zielposition
        error = measured_position - target_position

        deltaT = 1

        # Berechnung der PID Steuerung (nur P und D, ohne I-Term)
        if aktiv:
            currT = time.time()  # Aktuelle Zeit in Sekunden

            # deltaT in Sekunden berechnen
            deltaT = currT - self.prevT
            self.prevT = currT

        # Berechnung der Steuerabweichung
        speed_difference = (kp * error) + (kd * ((error - self.last_error) / deltaT))
        self.last_error = error

        # Berechne individuelle Motoren-Geschwindigkeiten
        self.left_speed = self.max_speed + speed_difference
        self.right_speed = self.max_speed - speed_difference

        # Beschränke die Geschwindigkeit auf den Bereich [0, max_speed]
        self.left_speed = max(0, min(self.left_speed, self.max_speed))
        self.right_speed = max(0, min(self.right_speed, self.max_speed))

    def get_left_speed(self):
    
        return self.left_speed
    
    def get_right_speed(self):

        return self.right_speed 