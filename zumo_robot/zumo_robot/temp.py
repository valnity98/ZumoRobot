import cv2
import numpy as np

# Webcam öffnen
cap = cv2.VideoCapture(2)


if not cap.isOpened():
    print("Fehler: Kamera konnte nicht geöffnet werden.")
    exit()

while True:
    # Frame von der Webcam einlesen
    ret, frame = cap.read()
    if not ret:
        print("Fehler: Frame konnte nicht gelesen werden.")
        break
    
    # Bestimmen der Höhe und Breite des Frames
    height, width = frame.shape[:2]

    # Definiere den horizontalen Streifen in der Mitte des Bildes
    y_start = height // 2 - 50  # 10 Pixel über der Mitte
    y_end = height // 2 + 50   # 10 Pixel unter der Mitte
    strip = frame[y_start:y_end, :]  # Der 20 Pixel hohe Streifen

    gray = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY)
    
    # Schritt 3: Schwellenwertanpassung, um schwarze Linien zu extrahieren
    # Hier verwenden wir einen hohen Schwellenwert, um den weißen Hintergrund zu erfassen
    _, thresholded = cv2.threshold(gray, 50, 100, cv2.THRESH_BINARY_INV)
    
    # Schritt 4: Konturen finden
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    


    if contours:
        # Größte Kontur finden
        largest_contour = max(contours, key=cv2.contourArea)

        # Berechnung der Momente der größten Kontur
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            # Berechnung des Schwerpunkts
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # Schwerpunkt als roten Punkt zeichnen
            cv2.circle(strip, (cX, cY), 5, (0, 0, 255), -1)  # Roter Punkt
            
            # Ausgabe der Koordinaten
            print(f"Schwerpunkt der größten Kontur: ({cX}, {cY + y_start})")  # Anpassen des Y-Werts

        # Zeichne die größte Kontur
        cv2.drawContours(strip, [largest_contour], -1, (0, 255, 0), 2)  # Grüne Kontur

    # Setze den bearbeiteten Streifen wieder in das Originalbild ein
    frame[y_start:y_end, :] = strip

    # Bild mit der größten Kontur und dem Schwerpunkt anzeigen
    cv2.imshow("Erkennung", strip)
    cv2.imshow("Threshold", thresholded)
    cv2.imshow("Original", frame)

    # Schleife mit der Taste 'q' beenden
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Resourcen freigeben
cap.release()
cv2.destroyAllWindows()
