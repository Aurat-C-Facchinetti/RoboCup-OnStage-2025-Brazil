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


def go_to_silvana():
    global robot_pos, robot_angle,offset

    # --- STEP 1: Rotate robot to face 180° ---
    angle_diff = (180 - robot_angle + 360) % 360 
    if angle_diff > 180:
        angle_diff -= 360 # Normalize to [-180, 180]
    robot_angle = move_rotation(angle_diff,ser,btsock,offset) # Perform rotation
    time.sleep(0.2)

    # --- STEP 2: Move forward 6 steps, each one is 20cm (in the 180° direction) ---
    for i in range(6):
        robot_angle=move_linear('forward', robot, 0,robot_pos, robot_angle,ser,btsock,offset)

    # --- STEP 3: Rotate robot to face 270° ---
    angle_diff = (270 - robot_angle + 360) % 360 
    if angle_diff > 180:
        angle_diff -= 360
    robot_angle = move_rotation(angle_diff,ser,btsock,offset)
    time.sleep(0.2)

    # --- STEP 4: Move forward 3 steps ---
    for i in range(3):
        robot_angle=move_linear('forward', robot, 0,robot_pos, robot_angle,ser,btsock,offset)

    # --- STEP 5: Rotate robot to face 0° ---
    angle_diff = (0 - robot_angle + 360) % 360 
    if angle_diff > 180:
        angle_diff -= 360
    robot_angle = move_rotation(angle_diff,ser,btsock,offset)
    time.sleep(0.2)

    # --- STEP 6: Move forward 4 steps ---
    # Final approach towards the target location (near Silvana)
    for i in range(4):
        robot_angle=move_linear('forward', robot, 0,robot_pos, robot_angle,ser,btsock,offset)


# Function to make the robot follow a given path (list of tiles coordinates)
def follow_path(path):
    global robot_pos, robot_angle,offset

    # Loop through all tiles in the path
    for (y, x) in path:
        while True:
            target_pos =fmap.mat_to_pixel(y,x) # Convert grid coordinates to image/pixel position
            # --- Compute the angle between the robot and the target ---
            angle_to_target = fmap.calcola_angolo(robot_pos, target_pos)
            angle_diff = (angle_to_target - robot_angle + 360) % 360
            if angle_diff > 180:
                angle_diff -= 360

            # --- Rotate the robot to face the target tile ---
            robot_angle = move_rotation(angle_diff,ser,btsock,offset)
            fmap.visualizza_mappa(robot_angle,robot_pos) # Update the map visualization
            cv2.waitKey(10)
            old=fmap.pixel_to_mat(robot_pos[0], robot_pos[1]) # Current tile before moving
            current_tile=old
            while current_tile==old and current_tile!=[y,x]:
                # While still in the same tile and not yet in the target tile, move forward
                robot_angle=move_linear('forward', robot, 0,robot_pos, robot_angle,ser,btsock,offset)
                fmap.visualizza_mappa(robot_angle,robot_pos)
                key = cv2.waitKey(10)
                current_tile = fmap.pixel_to_mat(robot_pos[0], robot_pos[1]) # Check if the robot entered a new tile

            # --- Target tile reached ---
            if current_tile == (y, x):
                print(f"Casella {y}, {x} raggiunta")
                break # Move on to the next tile in the path

# Function to move robotic arm to the side (prepare for interaction)
def posizione_laterale():
    ser.write(b'b000,') # b = base 
    time.sleep(0.5)
    ser.write(b'v150,') # v = shoulder
    time.sleep(0.5)
    ser.write(b'x090,') # x = wrist
    time.sleep(0.5)
    ser.write(b'c090,') # c = elbow

# Function to put back the object using robotic arm servos in the toolbox
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
    ser.write(b'z030,') # z = hand

# Function to pick up an object using the robotic arm
def prendi_oggetto():
    ser.write(b'b090,')
    time.sleep(0.5)
    ser.write(b'v150,')
    time.sleep(0.5)
    ser.write(b'c100,')
    time.sleep(0.5)
    ser.write(b'b090,')
    time.sleep(0.5)
    ser.write(b'x045,')
    time.sleep(0.5)
    ser.write(b'z000,')
    time.sleep(3)
    ser.write(b'z090,')

# Function to close the toolbox (cassetta) with the robotic arm
def chiudi_cassetta():
    ser.write(b'x070,')
    time.sleep(0.5)
    ser.write(b'z090,')
    time.sleep(0.5)
    ser.write(b'v165,')
    time.sleep(0.5)
    ser.write(b'b170,')
    time.sleep(0.5)
    ser.write(b'c070,')
    time.sleep(0.5)
    ser.write(b'b090,')

# Function to move the robotic arm into "driving position"
def posizione_guida():
    ser.write(b'v155,')
    time.sleep(0.5)
    ser.write(b'b080,')
    time.sleep(0.5)
    ser.write(b'c045,')
    time.sleep(0.5)
    ser.write(b'x030,')

#-------------------------------------------------------------
offset=0

robot=spider # Distance the robot moves with each step
btsock = connessione_bluetooth(HC05_MAC,PORT) # Establish Bluetooth connection

# Check the robot mode:
# 0 = only simulation/graphics
# 1 = serial communication (Silvestro)
# 2 = Bluetooth communication (Trash)
if robotm[0]==1:	#0 = solo grafica, 1 = seriale, 2 = bleutooth (trash)
    ser=init_connessione() #seriale
    ser.write(b'g,') # Request the current angle
    offset=read_angle_from_serial(ser) # Store the offset as the "zero" reference
print(offset)
robot_angle=0 # Initialize robot's angle

cam_thread = CameraThread()	# Start camera in a separate thread
cam_thread.start()

ser.write(b't000,') # Move the camera in a default position while waiting for a valid frame
# Wait until a valid frame is received from the camera
while True:
   color_image, depth_image = cam_thread.get_frames()
   if color_image is not None and depth_image is not None:
   	break
print("Sono pronto per partire")

try:
    modello('best.pt') # Load the letter recognition model
   
    print("Aspetto di vedere qualcuno...")
    ser.write(b't045,') # Move the robot head, with camera, to a neutral position
    # Wait for someone to approach the 3D camera to start the show
    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            center=[320,240]
            distance = calculate_distance(depth_image, center[0], center[1]) 
            if distance is not None and distance<50: # If somethong is close enough
                break
                
    posizione_laterale()      
    print("sono triste")
    
    # Sad head movement
    ser.write(b't080,')
    time.sleep(4)
    ser.write(b't045,')
    
    # ---- Working at the table ----
    for quanti in range(2): # Repeat for 2 objects
        time.sleep(2)


        while True:
            distance=None
            color_image, depth_image = cam_thread.get_frames()
            if color_image is not None and depth_image is not None:	#se c'è qualcosa usa yolo
                center=visualizza_yolo(color_image,"U")	# Try to detect letter U
                if center is not None:	#se esiste centro calcolca distanza con fotocamera 3d
                    distance = calculate_distance(depth_image, center[0], center[1])
                    take=1 # Hammer (U)
                else:
                    center=visualizza_yolo(color_image,"S") # Try to detect letter S
                    if center is not None:
                        distance = calculate_distance(depth_image, center[0], center[1])
                        take=2 # Spray (S)
                if distance is not None and distance<200: # Check if the detected letter is close enough
                    break
                cv2.waitKey(1)

        # Show object center on screen
        cv2.circle(color_image, center, 2 , (0, 255, 0), 2)
        cv2.imshow('Color Frame', color_image)
        cv2.waitKey(1);

        # Pick up the object
        prendi_oggetto()
        time.sleep(3)
        if take==1:	
            print("take 1")
            ser.write(b'm,') # Hammer
        else:
            print("take 2")
            ser.write(b'o,') # Spray
        time.sleep(15)

        # Put the object back
        posizione_laterale()
        time.sleep(5)
        riponi_oggetto() 
        time.sleep(5)
        posizione_laterale()
    
    time.sleep(2)
    chiudi_cassetta()
    posizione_laterale()
    
    # ---- Trash guide phase ----
    robotm[0]=2 # Activate Bluetooth-guided movement
    time.sleep(10)
    
    flush_buffer(btsock)
    invia_messaggio("b20",btsock) # Backward for 20 cm
    print(ricevi_messaggio(btsock))
   
    flush_buffer(btsock)
    invia_messaggio("1",btsock) # Move eyes
    print(ricevi_messaggio(btsock))
    flush_buffer(btsock)
    
    posizione_guida()

    # Climb the scooter
    ser.write(b'k,')
    time.sleep(10)

    go_to_silvana()
    
    # Arrived at destination, dismount, Silvestro moves away
    flush_buffer(btsock)
    invia_messaggio("1",btsock) 
    print(ricevi_messaggio(btsock))
    flush_buffer(btsock)
    
    posizione_laterale()
    time.sleep(5)
    ser.write(b'j,') # Dismount
    time.sleep(5)
    flush_buffer(btsock)
    invia_messaggio("f55",btsock) 
    print(ricevi_messaggio(btsock))
    invia_messaggio("2",btsock) # Final coreography of Trash
    print(ricevi_messaggio(btsock))
    
    # ---- Final phase ----
    # Turn west (towards Silvana) and walk forward a bit
    robotm[0]=1 # Back to serial/zampa movement
    punto_finale=[robot_pos[0],robot_pos[1]-40]	# Move 40 pixels to the left
    start = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
    goal = fmap.pixel_to_mat(punto_finale[0],punto_finale[1])
    path_to_start = fmap.a_star_search(start, goal, obstacle_map)
    if path_to_start:
        follow_path(path_to_start)

    # Deliver the rose
    time.sleep(2)
    ser.write(b'u,')
 
    
finally:
    input("Premi per terminare...")
    cv2.destroyAllWindows()
    chiudi_connessione(btsock)
    cam_thread.stop()
