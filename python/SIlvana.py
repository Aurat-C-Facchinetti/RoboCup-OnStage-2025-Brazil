from definizioni import *
from coordinateoggetto import *
from immagini import *
import funzionimappa as fmap
from move import *
import cv2
import numpy as np
import math
from braccio import *
from blue import *

# Funzione per seguire il percorso trovato
def follow_path(path):
    global robot_pos, robot_angle,offset

    for (y, x) in path:
        while True:
            target_pos =fmap.mat_to_pixel(y,x)
            # Calcola l'angolo verso la destinazione
            angle_to_target = fmap.calcola_angolo(robot_pos, target_pos)
            angle_diff = (angle_to_target - robot_angle + 360) % 360

            if angle_diff > 180:
                angle_diff -= 360

            # Ruota verso la destinazione
            robot_angle = move_rotation(angle_diff,ser,btsock,offset)
            fmap.visualizza_mappa(robot_angle,robot_pos)
            cv2.waitKey(10)
            old=fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
            current_tile=old
            while current_tile==old and current_tile!=[y,x]:
                # Muovi in avanti verso la destinazione
                robot_angle=move_linear('forward', robot, 0,robot_pos, robot_angle,ser,btsock,offset)
                fmap.visualizza_mappa(robot_angle,robot_pos)
                key = cv2.waitKey(10)
                # Controlla se il robot è nella casella target
                current_tile = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])

            if current_tile == (y, x):
                print(f"Casella {y}, {x} raggiunta")
                break


offset=0
robot=spider
btsock = 0
if robotm[0]==1:
    ser=init_connessione() #seriale
    ser.write(b'g,')
    offset=read_angle_from_serial(ser)#memorizzo il nuovo zero check leggi angolo
print(offset)
robot_angle=0
cam_thread = CameraThread()
cam_thread.start()
modello('yolov8s.pt')
try:
    '''
    #inizio aspetto lo start dato da un passaggio vicino alla fotocamera
    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            center=[320,240]
            distance = calculate_distance(depth_image, center[0], center[1])
            if distance is not None and distance<200:
                break
    #ruota in direzione del centro palco
    robot_angle = move_rotation(180,ser,btsock,offset)
    #cerco un uomo e ne calcolo la posizione sulla mappa


    input("Cerco uomo")

    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            center=visualizza_yolo(color_image,"person")
            if center is not None:
                print(center,distance)
                distance = calculate_distance(depth_image, center[0], center[1])

                print(center,distance)
            if distance is not None and distance<400:
                break
    punto_finale=dove_object(robot_pos,center[0],distance,robot_angle)
    #cerca il percorso e lo raggiunge
    start = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
    goal = fmap.pixel_to_mat(punto_finale[0],punto_finale[1])
    path_to_start = fmap.a_star_search(start, goal, obstacle_map)
    if path_to_start:
        follow_path(path_to_start)
    input("raggiunto")

    #leggo l'angolo
    ser.write(b'g,')
    robot_angle=read_angle_from_serial(ser)
    angle_diff = (270 - robot_angle + 360) % 360
    if angle_diff > 180:
        angle_diff -= 360
    #ruoto verso il pubblico
    robot_angle = move_rotation(angle_diff,ser,btsock,offset)
    #qui scena bicchiere
    ser.write(b'l,')
    time.sleep(2)
    ser.write(b'z000,')
    '''
    print("abbasso il braccio")
    ser.write(b'l,')
    input("vai")
    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            center = find_largest_green_area(color_image,lower_green,upper_green)
            distance = calculate_distance(depth_image, center[0], center[1])
            if distance is not None and distance<50:
                break

    cv2.circle(color_image, center, 2 , (0, 255, 0), 2)
    cv2.imshow('Color Frame', color_image)
    cv2.waitKey(0);
    #calcolo altezza da terra e angolo
    t_ang,t_alt=target(center,distance)
    print(t_ang,t_alt,distance)
    angoli=find_closest_configuration(distance+1,t_alt)
    print(angoli)
    draw_segments(angoli)

    muovi_braccio(ser,angoli)
    stringa = f"b{180-t_ang:03d},"  # Formatta il numero con tre cifre
    ser.write(stringa.encode())   # Converte la stringa in bytes e la invia
    time.sleep(1.5)
    ser.write(b'z060,')
    #porta il bicchiere alla bocca e lo riappoggia sul tavolo



    #finita scena bicchiere attendo che uomo prenda bicchiere
    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            center=[320,240]
            distance = calculate_distance(depth_image, center[0], center[1])
            if distance is not None and distance<200:
                break
    #mi giro a 0 gradi
    #leggo l'angolo
    ser.write(b'g,')
    robot_angle=read_angle_from_serial(ser)
    angle_diff = (0 - robot_angle + 360) % 360
    if angle_diff > 180:
        angle_diff -= 360
    #ruoto verso Spidy
    robot_angle = move_rotation(angle_diff,ser,btsock,offset)
    #testare se stare fermo o avvicinarsi

finally:
    cv2.destroyAllWindows()
    chiudi_connessione(btsock)
    cam_thread.stop()
