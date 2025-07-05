

import cv2
import numpy as np
import math
from collections import defaultdict

# Impostazioni iniziali
width, height = 300, 300  # Dimensione della finestra
center = (width // 2, height // 2)  # Centro della finestra
lengths = [17, 17, 25]  # Lunghezze dei segmenti
angles = [0, 0, 0]  # Angoli di rotazione dei segmenti
selected_segment = 0  # Segmento selezionato per la rotazione (0 = primo segmento)
pos_map = defaultdict(list)  # Struttura per memorizzare le posizioni possibili
altpav=46 #misura centro braccio a pavimento
point=[0,0]
def calculate_endpoint(center, lengths, angles):
    x, y = center  # Punto iniziale
    cumulative_angle = 0  # Angolo cumulativo inizializzato a zero

    for i in range(len(lengths)):  # Iterazione sugli indici
        cumulative_angle += angles[i]  # Aggiungo l'angolo corrente al cumulativo
        # Calcolo il nuovo punto finale
        x_new = x + lengths[i] * math.cos(math.radians(cumulative_angle + 90))
        y_new = y + lengths[i] * math.sin(math.radians(cumulative_angle + 90))
        # Aggiorno il punto di partenza per il prossimo segmento
        x, y = x_new, y_new

    return int(x), int(y)



# Funzione per generare tutte le posizioni possibili
def generate_position_map(center, lengths):
    pos_map = defaultdict(list)
    for a1 in range(90, 181, 1):
        for a2 in range(-90, 91, 1):#se metti a 0 il motore è in centro
            for a3 in range(0, 21, 1):
                angles = [a1, a2, a3]
                endpoint = calculate_endpoint(center, lengths, angles)
                if endpoint[0] < center[0]:  # Considera solo i punti a sinistra del centro
                    pos_map[endpoint].append(angles)  # Salviamo le configurazioni degli angoli
    return pos_map

def find_closest_configuration(dist_oggetto,alt_oggetto):
    global point
    point=(int(center[0]-dist_oggetto),int(center[1]+altpav-alt_oggetto-12))#prende distanza e altezza
    print(point)
    # Controlla se il punto è esattamente presente nel dizionario
    if point in pos_map:
        return pos_map[point][0]  # Ritorna la prima configurazione associata

    # Trova il punto più vicino iterando su tutti i punti
    closest_point = None
    min_distance = float('inf')
    for p in pos_map.keys():
        distance = math.dist(p, point)
        if distance < min_distance:
            min_distance = distance
            closest_point = p

    # Verifica se il punto più vicino è fuori portata
    if closest_point is None or min_distance > sum(lengths):
        return None
    # Ritorna la configurazione associata al punto più vicino
    return pos_map[closest_point][0]


# Funzione per disegnare i segmenti
def draw_segments(angles):
    global lengths,point
    image = np.zeros((height, width, 3), dtype=np.uint8)
    points = [center]
    for i, length in enumerate(lengths):
        angle = sum(angles[:i+1])  # Sommiamo gli angoli cumulativi fino al segmento corrente
        x1 = points[-1][0] + length * math.cos(math.radians(angle + 90))  # Aggiungiamo 90 gradi
        y1 = points[-1][1] + length * math.sin(math.radians(angle + 90))  # Aggiungiamo 90 gradi
        points.append((int(x1), int(y1)))

        # Disegniamo il segmento
        cv2.line(image, points[-2], points[-1], (255, 0, 0), 2)
    #disegno la base pavimento a circa xcm dal centro braccio
    pavstart=(center[0]-20,center[1]+altpav)
    pavend=(center[0]+20,center[1]+altpav)
    cv2.line(image, pavstart,pavend , (255, 255, 255), 2)
    #disegno sulla mappa laterale braccio l'oggetto
    cv2.circle(image, point, 2 , (0, 255, 0), 2)
    cv2.imshow("Segmenti Rotanti", image)
    cv2.waitKey(1)


#genera mappa posizioni
print("genero posizioni braccio")
pos_map = generate_position_map(center, lengths)
