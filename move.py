from definizioni import *
from blue import *
import funzionimappa as fmap
import math
import serial
import time


def init_connessione():
    try:
        ser = serial.Serial(porta, baudrate, timeout=1)
        print(f"Collegato a {porta} con baudrate {baudrate}")
        time.sleep(2)  # Attendi che la connessione si stabilisca
        return ser
    except serial.SerialException as e:
        print(f"Errore nella connessione seriale: {e}")

def muovi_braccio(ser,angoli):

            stringa = f"v{angoli[0]:03d},"  # Formatta il numero con tre cifre
            ser.write(stringa.encode())   # Converte la stringa in bytes e la invia
            time.sleep(1.5)
            stringa = f"c{angoli[1]+90:03d},"  # Formatta il numero con tre cifre
            ser.write(stringa.encode())   # Converte la stringa in bytes e la invia
            time.sleep(1.5)
            stringa = f"x{angoli[2]+90:03d},"  # Formatta il numero con tre cifre
            ser.write(stringa.encode())   # Converte la stringa in bytes e la invia
            time.sleep(1.5)
# Funzione per leggere l'angolo dalla seriale
def read_angle_from_serial(ser):
    angolo=0
    global offset

    while True:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                print(f"Ricevuto da seriale: {line}")  # Debug: dati grezzi
                if line.startswith("Yaw:"):
                    try:
                        angle = float(line.split(":")[1])
                        angolo = angle % 360  # Mantieni l'angolo tra 0 e 360
                        return angolo
                    except ValueError as e:
                        print(f"Errore nel parsing dell'angolo: {e}")
            except Exception as e:
                print(f"Errore durante la lettura della seriale: {e}")

def move_linear(direction,who,vel,robot_pos, robot_angle,ser,btsock,offset):
    print(direction)
    if robotm[0]==1:
        #comando arduino movimento seriale
        if direction=="forward":
            if vel==0:
                ser.write(b'w,')
            else:
                ser.write(b'W,')
        else:
            if vel==0:
                ser.write(b's,')
            else:
                ser.write(b'S,')
        robot_angle = read_angle_from_serial(ser)-offset
        if robot_angle<0:
        	robot_angle+=360
    elif robotm[0]==2:
        ser.write(b'b080,')	#braccio dritto
        if direction=="forward":
            flush_buffer(btsock)
            invia_messaggio("f20",btsock)
            stop=ricevi_messaggio(btsock)
        else:
            flush_buffer(btsock)
            invia_messaggio("b20",btsock)
            stop=ricevi_messaggio(btsock)
        #quando ricevo ok leggo angolo
        ser.write(b'g,')
        robot_angle = read_angle_from_serial(ser)-offset
        print(robot_angle)
        if robot_angle<0:
        	robot_angle+=360

    fmap.move_robot(direction, who, robot_pos, robot_angle)

    return robot_angle

def move_rotation(angle,ser,btsock,offset):
    global robot_angle
    angle=int(angle)
    strangolo=''
    if robotm[0]==1:
        strangolo = f"{abs(angle):03}"
        if angle>=0:
            strangolo = f"d{strangolo},"
            print(strangolo)
        else:
            strangolo = f"a{strangolo},"
            print(strangolo)
        ser.write(strangolo.encode())
        #attesa fine movimento
        robot_angle = read_angle_from_serial(ser)-offset
        if robot_angle<0:
        	robot_angle+=360
    elif robotm[0]==2:
        strangolo = f"{abs(angle)}"
        if angle>=0:
            strangolo = f"r{strangolo}"
            ser.write(b'b090,')	#sterza
            print(strangolo)
        else:
            strangolo = f"l{strangolo}"
            ser.write(b'b070,')	#sterza
            print(strangolo)
        flush_buffer(btsock)
        invia_messaggio(strangolo,btsock)
        stop=ricevi_messaggio(btsock)
        ser.write(b'g,')
        robot_angle = read_angle_from_serial(ser)-offset
        print(robot_angle)
        if robot_angle<0:
        	robot_angle+=360
        #quando finisce mi dice ok

    else:
        robot_angle+=angle
    return robot_angle
