from definizioni import *
import math
import cv2
import imutils
from queue import PriorityQueue


def visualizza_mappa(robot_angle,robot_pos):

    mappa_copy = mappa.copy()
    draw_robot(mappa_copy, robot_pos, robot_angle)
    cv2.imshow('Mappa con Robot', mappa_copy)



def move_robot(direction, step, robot_pos, robot_angle):
    angle_rad = math.radians(robot_angle)
    dx = int(step * math.cos(angle_rad))
    dy = int(step * math.sin(angle_rad))
    if direction == 'backward':
        dx = -dx
        dy = -dy
    new_x = robot_pos[0] + dx
    new_y = robot_pos[1] + dy
    if 0 <= new_x < map_width and 0 <= new_y < map_height:
        robot_pos[0] = new_x
        robot_pos[1] = new_y

# Funzione per calcolare l'angolo tra due punti
def calcola_angolo(p1, p2):
    delta_x = p2[0] - p1[0]
    delta_y = p2[1] - p1[1]
    angolo_rad = math.atan2(delta_y, delta_x)
    angolo_deg = math.degrees(angolo_rad)
    if angolo_deg < 0:
        angolo_deg = angolo_deg + 360
    return angolo_deg


# Funzione per ruotare un punto (utile per disegnare direzioni)
def ruota_punto(p1, angolo, distanza):
    angolo_attuale = 0
    nuovo_angolo = angolo_attuale + math.radians(angolo)
    nuovo_x = int(p1[0] + distanza * math.cos(nuovo_angolo))
    nuovo_y = int(p1[1] + distanza * math.sin(nuovo_angolo))
    return nuovo_x, nuovo_y


# Funzione per disegnare il robot
def draw_robot(mappa, pos, angle):
    global robot_radius
    cx, cy = pos
    cv2.circle(mappa, (cx, cy), robot_radius, (0, 255, 0), -1)
    raggio_length = robot_radius * 0.8
    x_end, y_end = ruota_punto(pos, angle, raggio_length)
    cv2.line(mappa, (cx, cy), (x_end, y_end), (0, 0, 255), 2)

# Funzione per disegnare le distanze e aggiornare la mappa degli ostacoli
def draw_distances(mappa, robot_pos, robot_angle, distances_cm):
    scale_cm_to_px = 1
    num_distances = len(distances_cm)
    angle_step = 180 / (num_distances - 1)
    for i, distance_cm in enumerate(distances_cm):
        angle = robot_angle - 90 + i * angle_step
        angle_rad = math.radians(angle)
        distance_px = int((distance_cm + robot_radius) * scale_cm_to_px)
        point_x = int(robot_pos[0] + distance_px * math.cos(angle_rad))
        point_y = int(robot_pos[1] + distance_px * math.sin(angle_rad))
        if 0 <= point_x < map_width and 0 <= point_y < map_height:
            cv2.circle(mappa, (point_x, point_y), 1, (0, 0, 0), -1)

    obstacle_map=matrice_ostacoli(mappa)
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)
    return obstacle_map

def pixel_to_mat(x_pixel, y_pixel):
    # Poiché c'è un offset di 20 pixel tra le celle
    x_goal = (x_pixel) // 20
    y_goal = (y_pixel) // 20
    return (y_goal, x_goal)

def mat_to_pixel(y,x):
    return (x*20+10,y*20+10)

def trovanero(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    (T, thresh) = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
    cnts = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)#problemi di compatibilità
    if len(cnts)!=0:
        print("0")
        return 0
    else:
        print("1")
        return 1

#creazione matrice con 1 casella vuota e 0 casella con ostacolo
def matrice_ostacoli(mappa):
    cx=0
    cy=0
    mat = np.zeros((58,58), dtype=np.uint8)
    for y in range(0,1160,20):
        for x in range(0,1160,20):
            taglio=mappa[y:y+40,x:x+40]
            mat[cy,cx]=trovanero(taglio)
            cx+=1
        cy+=1
        cx=0
    return mat

def distanza_2p(p1,p2):
    # Calcola le differenze tra le coordinate di p1 e p2
    delta_x = p1[0] - p2[0]
    delta_y = p1[1] - p2[1]
    # Calcola la distanza tra p1 e p2
    distanza = math.sqrt(delta_x**2 + delta_y**2)
    return distanza

def a_star_search(start, goal, obstacle_map):
    num_rows, num_cols = obstacle_map.shape  # Ottieni dimensioni della mappa

    def heuristic(a, b):
        """ Calcola la distanza di Manhattan tra due punti """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # Controllo se partenza o arrivo sono ostacoli
    if obstacle_map[start[0], start[1]] == 0 or obstacle_map[goal[0], goal[1]] == 0:
        print("Partenza o arrivo sono ostacoli!")
        return -1

    # Inizializza la coda prioritaria, la mappa dei costi e il tracciamento del percorso
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    # Movimenti possibili: 4 direzioni cardinali + 4 diagonali
    moves = [
        (-1, 0), (1, 0), (0, -1), (0, 1),  # Alto, basso, sinistra, destra
        (-1, -1), (-1, 1), (1, -1), (1, 1)  # Diagonali
    ]

    while not frontier.empty():
        _, current = frontier.get()

        if current == goal:
            break  # Obiettivo raggiunto

        for dy, dx in moves:
            next_pos = (current[0] + dy, current[1] + dx)

            if 0 <= next_pos[0] < num_rows and 0 <= next_pos[1] < num_cols:
                cell_value = obstacle_map[next_pos[0], next_pos[1]]
                if cell_value == 0:
                    continue  # Evita completamente le celle ostacolo

                move_cost = math.sqrt(2) if dy != 0 and dx != 0 else 1
                new_cost = cost_so_far[current] + move_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current

    # Ricostruzione del percorso
    path = []
    current = goal

    while current != start:
        if current is None:
            print("Nessun percorso trovato!")
            return -2

        path.append(current)
        current = came_from.get(current)

    path.reverse()

    return path
