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

# Function to follow the calculated path
def follow_path(path):
    global robot_pos, robot_angle,offset

    for (y, x) in path:
        input()
        while True:
        
            target_pos =fmap.mat_to_pixel(y,x)
            # Calculate the angle toward the destination
            angle_to_target = fmap.calcola_angolo(robot_pos, target_pos)
            angle_diff = (angle_to_target - robot_angle + 360) % 360

            if angle_diff > 180:
                angle_diff -= 360

            # Rotate toward the destination
            robot_angle = move_rotation(angle_diff,ser,btsock,offset)
            fmap.visualizza_mappa(robot_angle,robot_pos)
            cv2.waitKey(10)
            old=fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
            current_tile=old
            while current_tile==old and current_tile!=[y,x]:
                # Move forward toward the destination
                robot_angle=move_linear('forward', robot, 0,robot_pos, robot_angle,ser,btsock,offset)
                fmap.visualizza_mappa(robot_angle,robot_pos)
                key = cv2.waitKey(10)
                # Check if the robot has reached the target tile
                current_tile = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])

            if current_tile == (y, x):
                print(f"Casella {y}, {x} raggiunta")
                break

# Function to move the robot arm to the lateral (side) position
def posizione_laterale():
    ser.write(b'b000,') # Rotate base (b = base) to 0 degrees
    time.sleep(0.5)
    ser.write(b'v150,') # Move shoulder joint (v = shoulder) to 150 degrees
    time.sleep(0.5)
    ser.write(b'x090,') # Move wrist joint (x = wrist) to 90 degrees
    time.sleep(0.5)
    ser.write(b'c090,') # Move elbow joint (c = elbow) to 90 degrees
    
# Sequence of commands to move the robot arm to place an object in the toolbox
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
    ser.write(b'z030,') # Open the hand (z = hand) to release the object
    time.sleep(0.5)
    
# Sequence of commands to move the robot arm to pick up an object received from the actor
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
    
# Sequence of commands to close the toolbox using the robot arm
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
    
# Move robot arm joints to the "guide" position (a predefined pose to reach while moving with Trash)
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
btsock = connessione_bluetooth(HC05_MAC,PORT)	# Establish Bluetooth connection using MAC and port

if robotm[0]==1:	# Mode check: 0 = graphics only, 1 = serial communication, 2 = Bluetooth (Trash)
    ser=init_connessione()
    ser.write(b'g,')	
    offset=read_angle_from_serial(ser)	# Read and store the zero angle offset for calibration
print(offset)
robot_angle=0
# Initialize and start the camera thread
cam_thread = CameraThread()	
cam_thread.start()

try:
    modello('best.pt') # Load the letter recognition model
    # Move the robot's head to express sadness
    ser.write(b't080,') #t = head
    time.sleep(4)
    ser.write(b't045,')
    
    ser.write(b'c040,')
    time.sleep(0.5)
    posizione_laterale()
    
    # ----- Work at the table -----
    for quanti in range(2):
        time.sleep(2)
        while True:
            distance=None
            color_image, depth_image = cam_thread.get_frames()
            if color_image is not None and depth_image is not None:	# If frames are valid, proceed with yolo
                center=visualizza_yolo(color_image,"U")	# Try to detect letter 'U' and get its center
                if center is not None:	# If letter 'U' is detected
                    distance = calculate_distance(depth_image, center[0], center[1])
                    take=1 # Mark that the target is 'U'
                else:
                    center=visualizza_yolo(color_image,"S") # Otherwise, try to detect letter 'S'
                    if center is not None:
                        distance = calculate_distance(depth_image, center[0], center[1])
                        take=2 # Mark that the target is 'S'
                if distance is not None and distance<200:
                    break
                cv2.waitKey(1)

        # Visualize the detected center point on the color frame
        cv2.circle(color_image, center, 2 , (0, 255, 0), 2)
        cv2.imshow('Color Frame', color_image)
        cv2.waitKey(1);

        prendi_oggetto() # Command robot to pick up the object
        time.sleep(3)
        if take==1:	# If letter 'U' was detected (hammer)
            print("take 1")
            ser.write(b'm,') # Send command 'm' (hammer action)
        else:	# If letter 'S' was detected (spray)
            print("take 2")
            ser.write(b'o,') # Send command 'o' (spray action)
        time.sleep(15)
        posizione_laterale()
        time.sleep(5)
        riponi_oggetto() # Command robot to put the object back
        time.sleep(5)
        posizione_laterale()
    
    time.sleep(2)
    chiudi_cassetta() # Command to close the toolbox
    posizione_laterale()

    # ----- Trash guidance -----
    robotm[0]=2	# Activate "trash" guidance mode
    time.sleep(10)

    # Communication with the robot via Bluetooth
    flush_buffer(btsock) # Clear the Bluetooth buffer
    invia_messaggio("b20",btsock) # Send command "b20" (go backword for 20cm)
    print(ricevi_messaggio(btsock))
    
    flush_buffer(btsock)
    invia_messaggio("1",btsock) # Send command "1" (eye movement)
    print(ricevi_messaggio(btsock))
    flush_buffer(btsock)
    

    # Climb onto the scooter
    ser.write(b'k,') # Send command to perform climbing motion
    time.sleep(10)
    
    # Calculate the destination point
    punto_finale=[robot_pos[0]-200,robot_pos[1]-100]
    # Convert pixel coordinates to map coordinates for pathfinding
    start = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
    goal = fmap.pixel_to_mat(punto_finale[0],punto_finale[1])
    # Use A* algorithm to find a path from current position to the goal
    path_to_start = fmap.a_star_search(start, goal, obstacle_map)
    print(path_to_start)
    posizione_guida() # Move the robot arm into the driving/steering position
    
    # If a valid path was found, follow it
    if path_to_start:
        follow_path(path_to_start)                                                   
    
    #arrivati al punto scende e Spidy si sposta
    flush_buffer(btsock)
    invia_messaggio("1",btsock) 
    print(ricevi_messaggio(btsock))
    flush_buffer(btsock)
    
    posizione_laterale()
    time.sleep(5)
    ser.write(b'j,') # Send command to dismount
    time.sleep(5)
    flush_buffer(btsock)
    invia_messaggio("f55",btsock) # Final Bluetooth command (go foward for 55cm)
    print(ricevi_messaggio(btsock))
    
    # ----- Final scene -----
    # The robot turns in her direction and walks closer
    robotm[0]=1 # Switch back to serial control (walking on legs)
    punto_finale=[robot_pos[0],robot_pos[1]+40]	# Define target point
    start = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
    goal = fmap.pixel_to_mat(punto_finale[0],punto_finale[1])
    path_to_start = fmap.a_star_search(start, goal, obstacle_map)
    if path_to_start:
        follow_path(path_to_start)

    time.sleep(2)
    ser.write(b'u,') # Command to pick up the rose
finally:
    cv2.destroyAllWindows()
    chiudi_connessione(btsock)
    cam_thread.stop()
