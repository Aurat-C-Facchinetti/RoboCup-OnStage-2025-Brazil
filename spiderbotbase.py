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
btsock = connessione_bluetooth(HC05_MAC,PORT)
if robotm[0]==1:
    ser=init_connessione() #seriale
    ser.write(b'g,')
    offset=read_angle_from_serial(ser)#memorizzo il nuovo zero check leggi angolo
print(offset)
robot_angle=0
cam_thread = CameraThread()
cam_thread.start()
try:
    while True:
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
        # Mostra i frame
            cv2.imshow('Color Frame', color_image)
        fmap.visualizza_mappa(robot_angle,robot_pos)
        key = cv2.waitKey(10)
        if key == ord('b'):  # test yolo
            modello('yolov8s.pt')
            print(visualizza_yolo(color_image,"clock"))
        if key == ord('l'):  # Mostra le distanze e disegna un ostacolo
            obstacle_map=fmap.draw_distances(mappa, robot_pos, robot_angle, distances)
            print(obstacle_map)
        elif key == ord('w'):
            robot_angle=move_linear('forward',robot, 0,robot_pos, robot_angle,ser,btsock,offset)
        elif key == ord('s'):
            robot_angle=move_linear('backward',robot, 0, robot_pos, robot_angle,ser,btsock,offset)
        elif key == ord('a'):
            robot_angle = move_rotation(-5,ser,btsock,offset)
        elif key == ord('d'):
            robot_angle = move_rotation(+5,ser,btsock,offset)
        elif key == ord('r'):
            # Ottieni il goal dalla posizione in pixel
            goal = fmap.pixel_to_mat(100,100)
            # Ottieni la posizione attuale del robot (in pixel) e calcola il percorso per tornare al punto iniziale
            start = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
            print(start,goal)
            path_to_start = fmap.a_star_search(start, goal, obstacle_map)
            print(path_to_start)
            if path_to_start:
                follow_path(path_to_start)
        elif key == ord ('t'):
            ser.write(b'l,')
            cv2.waitKey(2000)
            ser.write(b'z000,')

            center = find_largest_green_area(color_image,lower_green,upper_green)
            # Calcola la distanza con la media locale
            distance = calculate_distance(depth_image, center[0], center[1])
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
        elif key == ord ('h'):
            ser.write(b'l,')
            ser.write(b't120,')
            cv2.waitKey(0)
            stringa = f"b{180-t_ang:03d},"  # Formatta il numero con tre cifre
            ser.write(stringa.encode())   # Converte la stringa in bytes e la invia
            time.sleep(1.5)
            angoli=[180,0,90] #2 e terzo +90
            muovi_braccio(ser,angoli)
        elif key == ord('h'):#parte che raggiunge oggetto con bt, il primo valore è la x il secondo è la distanza.
            puntoobj=dove_object(robot_pos,150,100,robot_angle)

        elif key == ord('q'):
            break
finally:
    cv2.destroyAllWindows()
    chiudi_connessione(btsock)
    cam_thread.stop()
