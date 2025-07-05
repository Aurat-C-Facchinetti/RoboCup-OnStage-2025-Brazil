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
import time

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

#funzione per spostare lateralmente il braccio prima all'inizio dello spettacolo
def posizione_laterale():
    ser.write(b'b000,')
    time.sleep(0.5)
    ser.write(b'v150,')
    time.sleep(0.5)
    ser.write(b'x090,')
    time.sleep(0.5)
    ser.write(b'c090,')
    
def riponi_oggetto():
    ser.write(b'b000,')
    time.sleep(0.5)
    ser.write(b'v150,')
    time.sleep(0.5)
    ser.write(b'c090,')
    time.sleep(0.5)
    ser.write(b'x090,')
    time.sleep(0.5)
    ser.write(b'b110,')
    time.sleep(0.5)
    ser.write(b'x045,')
    time.sleep(0.5)
    ser.write(b'z030,')
    time.sleep(0.5)
    
def prendi_oggetto():
    ser.write(b'b090,')
    time.sleep(0.5)
    ser.write(b'v150,')
    time.sleep(0.5)
    ser.write(b'c090,')
    time.sleep(0.5)
    ser.write(b'b090,')
    time.sleep(0.5)
    ser.write(b'x045,')
    time.sleep(0.5)
    ser.write(b'z000,')
    time.sleep(3)
    ser.write(b'z090,')
    
def chiudi_cassetta():
    ser.write(b'x070,')
    time.sleep(0.5)
    ser.write(b'z090,')
    time.sleep(0.5)
    ser.write(b'v165,')
    time.sleep(0.5)
    #va alzata lo telecamera
    ser.write(b'b170,')
    time.sleep(0.5)
    ser.write(b'c070,')
    time.sleep(0.5)
    ser.write(b'b090,')
    
def posizione_guida():
    ser.write(b'v155,')
    time.sleep(0.5)
    ser.write(b'b080,')
    time.sleep(0.5)
    ser.write(b'c045,')
    time.sleep(0.5)
    ser.write(b'x030,')

offset=0
robot=spider	#quanto si muove per un passo
btsock = connessione_bluetooth(HC05_MAC,PORT)	#per comunicazine bluetooth
if robotm[0]==1:	#0 = solo grafica, 1 = seriale, 2 = bleutooth (trash)
    ser=init_connessione() #seriale
    ser.write(b'g,')	#gradi per offset
    offset=read_angle_from_serial(ser)	#memorizzo il nuovo zero check leggi angolo
print(offset)
robot_angle=0
cam_thread = CameraThread()	#per avvio fotocamera
cam_thread.start()
try:
    modello('best.pt')	#carcicamento modello riconoscimento lettere
    
    #movimento testa triste
    ser.write(b't030,')
    time.sleep(4)
    ser.write(b't000,')
    
    ser.write(b'c040,')
    time.sleep(0.5)
    posizione_laterale()
    '''
    #-----lavoro al tavolo------
    for quanti in range(2):
        time.sleep(2)


        while True:
            distance=None
            color_image, depth_image = cam_thread.get_frames()
            if color_image is not None and depth_image is not None:	#se c'è qualcosa usa yolo
                center=visualizza_yolo(color_image,"U")	#cerca centro della u
                if center is not None:	#se esiste centro calcolca distanza con fotocamera 3d
                    distance = calculate_distance(depth_image, center[0], center[1])
                    take=1
                else:
                    center=visualizza_yolo(color_image,"S")
                    if center is not None:
                        distance = calculate_distance(depth_image, center[0], center[1])
                        take=2
                if distance is not None and distance<200:
                    break
                    # Mostra il frame annotato
                cv2.waitKey(1)

        cv2.circle(color_image, center, 2 , (0, 255, 0), 2)
        cv2.imshow('Color Frame', color_image)
        cv2.waitKey(1);

        prendi_oggetto()
        time.sleep(5.0)
        if take==1:	#lettera u = martello
            print("take 1")
            ser.write(b'm,')
        else:	#lettera s = spray
            print("take 2")
            ser.write(b'o,')
        time.sleep(15)
        posizione_laterale()
        time.sleep(5)
        riponi_oggetto() 
        time.sleep(5)
        posizione_laterale()
    
    time.sleep(2)
    chiudi_cassetta()
    
    posizione_laterale()
    '''
    
    
    #-----guida trash-----
    robotm[0]=2	#attiviamo la guida di trash
    time.sleep(3)
    flush_buffer(btsock)
    invia_messaggio("b20",btsock)
    print(ricevi_messaggio(btsock))
    flush_buffer(btsock)
    invia_messaggio("1",btsock)
    flush_buffer(btsock)
    posizione_guida()

    #salita scooter
    ser.write(b'k,')
    time.sleep(10)
    #viaggiano verso il punto desiderato
    punto_finale=[robot_pos[0]-200,robot_pos[1]+100]
    start = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
    goal = fmap.pixel_to_mat(punto_finale[0],punto_finale[1])
    path_to_start = fmap.a_star_search(start, goal, obstacle_map)
    print(path_to_start)
    #BRACCIO SUL VOLANTE
    if path_to_start:
        follow_path(path_to_start)

    #arrivati al punto scende e Spidy si sposta

    invia_messaggio("1",btsock)
    
    flush_buffer(btsock)
    
    posizione_inizio_spettacolo()
    time.sleep(5)
    ser.write(b'j,')
    time.sleep(5)
    flush_buffer(btsock)
    invia_messaggio("f55",btsock)	#SPOSTAMENTO TRASH
    print(ricevi_messaggio(btsock))
    
    
    #-----finale-----
    #ruota in direzione ovest dove in teoria dovrebbe esserci lei e cammina fino ad avvicinarsi
    robotm[0]=1#ritorna la guida seriale sulle zampe
    punto_finale=[robot_pos[0],robot_pos[1]-40]	#vai a sinistra di 40 pixel
    start = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
    goal = fmap.pixel_to_mat(punto_finale[0],punto_finale[1])
    path_to_start = fmap.a_star_search(start, goal, obstacle_map)
    if path_to_start:
        follow_path(path_to_start)

    
    #prende la rosa
    time.sleep(1)
    ser.write(b'u,')
finally:
    cv2.destroyAllWindows()
    chiudi_connessione(btsock)
    cam_thread.stop()
