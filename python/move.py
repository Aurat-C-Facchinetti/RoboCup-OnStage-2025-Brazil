from definizioni import *
from blue import *
import funzionimappa as fmap
import math
import serial
import time

# Initialize serial connection to the robot
def init_connessione():
    try:
        ser = serial.Serial(porta, baudrate, timeout=1)
        print(f"Collegato a {porta} con baudrate {baudrate}")
        time.sleep(2)  # Wait for the connection to fully initialize
        return ser
    except serial.SerialException as e:
        print(f"Errore nella connessione seriale: {e}")

# Move the robotic arm based on a list of joint angles
def muovi_braccio(ser,angoli):
    # Joint 'v' = shoulder
    stringa = f"v{angoli[0]:03d},"  # Format the number with three digits
    ser.write(stringa.encode())   # Convert the string to bytes and send it
    time.sleep(1.5)
    # Joint 'c' = elbow (offset by +90)
    stringa = f"c{angoli[1]+90:03d},"  
    ser.write(stringa.encode())   
    time.sleep(1.5)
    # Joint 'x' = wrist (offset by +90)
    stringa = f"x{angoli[2]+90:03d}," 
    ser.write(stringa.encode())   
    time.sleep(1.5)
# Read the robot's angle from the serial port
def read_angle_from_serial(ser):
    angolo=0
    global offset

    while True:
        if ser and ser.in_waiting > 0: # Check if serial port is ready and has data
            try:
                line = ser.readline().decode('utf-8').strip()
                print(f"Ricevuto da seriale: {line}")  # Debug: raw data output
                if line.startswith("Yaw:"): # Check if the line contains yaw angle data
                    try:
                        angle = float(line.split(":")[1]) # Extract and convert the angle
                        angolo = angle % 360  # Normalize the angle to be within 0–359°
                        return angolo
                    except ValueError as e:
                        print(f"Errore nel parsing dell'angolo: {e}") # If parsing fails, print error
            except Exception as e:
                print(f"Errore durante la lettura della seriale: {e}") # Catch and display read errors
# Move robot forward or backward depending on control mode (serial or Bluetooth)
def move_linear(direction,who,vel,robot_pos, robot_angle,ser,btsock,offset):
    print(direction)
    if robotm[0]==1:
        # Send movement command to Arduino via serial
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
        # Read angle after movement, adjusting for offset
        robot_angle = read_angle_from_serial(ser)-offset 
        if robot_angle<0:
        	robot_angle+=360
    elif robotm[0]==2: # Mode 2: Bluetooth control
        ser.write(b'b080,')	#braccio dritto
        # Send Bluetooth command based on direction
        if direction=="forward":
            flush_buffer(btsock)
            invia_messaggio("f20",btsock)
            stop=ricevi_messaggio(btsock) # Wait for confirmation
        else:
            flush_buffer(btsock)
            invia_messaggio("b20",btsock)
            stop=ricevi_messaggio(btsock)
        # After receiving confirmation, read new angle
        ser.write(b'g,')
        robot_angle = read_angle_from_serial(ser)-offset
        print(robot_angle)
        if robot_angle<0:
        	robot_angle+=360
    # Update robot position in the map
    fmap.move_robot(direction, who, robot_pos, robot_angle)

    return robot_angle # Return updated orientation

# Rotate the robot by a given angle
def move_rotation(angle,ser,btsock,offset):
    global robot_angle
    angle=int(angle)
    strangolo='' # Command string to be sent
    if robotm[0]==1: # Mode 1: Serial movement
        strangolo = f"{abs(angle):03}" # Format angle with 3 digits (e.g. 045)
        if angle>=0:
            strangolo = f"d{strangolo}," # Clockwise rotation
            print(strangolo)
        else:
            strangolo = f"a{strangolo}," # Counterclockwise rotation
            print(strangolo)
        ser.write(strangolo.encode())
        # Wait and read the updated angle
        robot_angle = read_angle_from_serial(ser)-offset
        if robot_angle<0:
        	robot_angle+=360
    elif robotm[0]==2: # Mode 2: Bluetooth control
        strangolo = f"{abs(angle)}" # No need to zero-pad
        if angle>=0:
            strangolo = f"r{strangolo}" # Clockwise rotation
            ser.write(b'b090,')	# Turn arm to right
            print(strangolo)
        else:
            strangolo = f"l{strangolo}"
            ser.write(b'b070,')	# Turn arm to left
            print(strangolo)
        flush_buffer(btsock)
        invia_messaggio(strangolo,btsock) # Send command via Bluetooth
        stop=ricevi_messaggio(btsock)
        # Request and read new angle from Arduino
        ser.write(b'g,')
        robot_angle = read_angle_from_serial(ser)-offset
        print(robot_angle)
        if robot_angle<0:
        	robot_angle+=360
        #quando finisce mi dice ok

    else:
        robot_angle+=angle
    return robot_angle
