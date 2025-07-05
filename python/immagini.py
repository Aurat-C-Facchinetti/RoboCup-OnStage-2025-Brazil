import cv2
import numpy as np
from definizioni import *
from ultralytics import YOLO

model=None

def modello(strmodello):
    global model
    # Carica il modello YOLOv8s
    model = YOLO(strmodello)

def visualizza_yolo(frame, target_class):
    results = model(frame)
    max_area = 0
    center = None

    # Itera sui risultati per trovare l'oggetto più grande della classe specificata
    for result in results:
        boxes = result.boxes  # Contiene le informazioni sulle bounding box
        for box in boxes:
            # Estrai le coordinate della bounding box e altri dettagli
            x1, y1, x2, y2 = box.xyxy[0]
            conf = box.conf[0]
            cls = box.cls[0]
            label = model.names[int(cls)]

            # Controlla se la classe corrisponde a quella specificata
            if label == target_class:
                # Calcola l'area della bounding box
                area = (x2 - x1) * (y2 - y1)

                # Se l'area è maggiore della massima trovata finora, aggiorna il centro
                if area > max_area:
                    max_area = area
                    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                # Disegna la bounding box e l'etichetta sul frame
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f'{label}: {conf:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.imshow('immagine', frame)
                cv2.waitKey(1)


    # Ritorna il centro dell'oggetto più grande della classe specificata (o None se non trovato)
    return center


# Funzione per individuare la zona colorata più grande e mostrare la maschera usata nei test
def find_largest_green_area(color_image,lower_col,upper_col):
    # Convertire l'immagine in HSV
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Maschera per il verde
    mask = cv2.inRange(hsv_image, lower_col, upper_col)

    # Mostra la maschera per il verde
    #cv2.imshow('Green Mask', mask)

    # Trova i contorni
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, None

    # Trova il contorno con l'area massima
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)

    if M["m00"] > 0:
        # Calcola il centro
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return (cx, cy)
    return None, None
