from definizioni import *
from coordinateoggetto import *
from immagini import *
import funzionimappa as fmap
from move import *
import cv2
import numpy as np
import math
from braccio import *


# Function to follow the calculated path
def follow_path(path):
    global robot_pos, robot_angle, offset

    for (y, x) in path:
        while True:
            target_pos = fmap.mat_to_pixel(y, x)
            
            # Calculate the angle toward the destination
            angle_to_target = fmap.calcola_angolo(robot_pos, target_pos)
            angle_diff = (angle_to_target - robot_angle + 360) % 360

            if angle_diff > 180:
                angle_diff -= 360

            # Rotate toward the destination
            robot_angle = move_rotation(angle_diff, ser, btsock, offset)
            fmap.visualizza_mappa(robot_angle, robot_pos)
            cv2.waitKey(10)

            old = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
            current_tile = old

            while current_tile == old and current_tile != [y, x]:
                # Move forward toward the destination
                robot_angle = move_linear('forward', robot, 0, robot_pos, robot_angle, ser, btsock, offset)
                fmap.visualizza_mappa(robot_angle, robot_pos)
                key = cv2.waitKey(10)

                # Check if the robot has reached the target tile
                current_tile = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])

            if current_tile == (y, x):
                print(f"Tile {y}, {x} reached")
                break
# Function to move the robot arm to the lateral (side) position
def posizione_laterale():
    ser.write(b'b000,')   # Rotate base to 0 degrees
    time.sleep(0.5)
    ser.write(b'v150,')   # Move shoulder joint to 150 degrees
    time.sleep(0.5)
    ser.write(b'x090,')   # Move wrist joint to 90 degrees
    time.sleep(0.5)
    ser.write(b'c090,')   # Move elbow joint to 90 degrees
    time.sleep(0.5)

offset=0
robot=spider
btsock = 0
if robotm[0]==1:
    ser=init_connessione() # Initialize serial connection
    ser.write(b'g,')
    offset=read_angle_from_serial(ser)# Store the new zero reference (read angle)
time.sleep(3)
print(offset)
robot_angle=0
cam_thread = CameraThread()
cam_thread.start()
modello('yolov8s.pt')
try:
    posizione_laterale()   

    # Start: wait for someone to pass in front of the camera to trigger the action
    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            center=[320,240] # Check the center of the image
            distance = calculate_distance(depth_image, center[0], center[1])
            if distance is not None and distance<50:
                break
    # Rotate towards the center of the stage
    robot_angle = move_rotation(180,ser,btsock,offset)
    # Look for a person and calculate their position on the map
    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            # Use YOLO to detect a person in the image
            center=visualizza_yolo(color_image,"person")
            if center is not None:
                print(center,distance)
                # Measure the distance to the detected person
                distance = calculate_distance(depth_image, center[0], center[1])
                print(center,distance)
            # If a person is detected within 400mm, stop searching
            if distance is not None and distance<400:
                break
    # Calculate the target point on the map where the person is located
    punto_finale=dove_object(robot_pos,center[0],distance,robot_angle)
    
    # Find and follow the path to reach the person
    start = fmap.pixel_to_mat(robot_pos[0], robot_pos[1])
    goal = fmap.pixel_to_mat(punto_finale[0],punto_finale[1])
    path_to_start = fmap.a_star_search(start, goal, obstacle_map)
    if path_to_start:
        follow_path(path_to_start)

    time.sleep(4)
    
    # Read the current angle from the robot
    ser.write(b'g,')
    robot_angle=read_angle_from_serial(ser)
    angle_diff = (270 - robot_angle + 360) % 360
    if angle_diff > 180:
        angle_diff -= 360
    
    # Rotate the robot by the calculated angle difference to face the audience
    robot_angle = move_rotation(angle_diff,ser,btsock,offset)
    
    # Handling the glass
    posizione_laterale()
    time.sleep(3)
    ser.write(b'z000,')
    time.sleep(3)
    
    modello('best.pt') # Load the YOLO model for object detection
    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            center=visualizza_yolo(color_image,"H") # Detect the H and get its center coordinates
            if center is not None:
                distance = calculate_distance(depth_image, center[0], center[1]) # Compute distance from depth image
                if distance is not None and distance<100:
                    break
    # Mark the detected center on the color image and display it
    cv2.circle(color_image, center, 2 , (0, 255, 0), 2)
    cv2.imshow('Color Frame', color_image)
    cv2.waitKey(1);
    # Calculate target height from the ground and angle from the detected position and distance
    t_ang,t_alt=target(center,distance)
    print(t_alt)
    t_alt += 10
    print(t_ang,t_alt,distance)
    angoli=find_closest_configuration(distance,t_alt) # Find the best arm joint configuration for reaching the target
    print(angoli)
    draw_segments(angoli) # Visualize the arm configuration segments
    
    muovi_braccio(ser,angoli)
    # Format the angle command as a three-digit number and send it
    stringa = f"b{t_ang:03d}," 
    ser.write(stringa.encode())   
    time.sleep(1.5)
    ser.write(b'z057,')
    

    time.sleep(1.5)
    
    # Move the glass to the mouth and then place it back on the table through a sequence of servo commands
    ser.write(b'f180,')
    ser.write(b'c090,') 
    ser.write(b'v140,') 
    ser.write(b'c120,')
    ser.write(b'x130,')
    time.sleep(12)

    ser.write(b'c100,')
    ser.write(b'v150,')
    time.sleep(1)

    ser.write(b'c070,')
    ser.write(b'v160,')
    time.sleep(1)

    ser.write(b'c035,')
    ser.write(b'x080,')
    time.sleep(1)
    ser.write(b'f090,')
    ser.write(b'z000,')
    posizione_laterale()
    time.sleep(5)

    # End of the glass scene: wait for the person to take the glass
    while True:
        distance=None
        color_image, depth_image = cam_thread.get_frames()
        if color_image is not None and depth_image is not None:
            center=[320,240]
            distance = calculate_distance(depth_image, center[0], center[1])
            if distance is not None and distance<20: # Wait until the detected object (person's hand) is close enough
                break

    ser.write(b'g,')
    robot_angle=read_angle_from_serial(ser)
    angle_diff = (0 - robot_angle + 360) % 360
    if angle_diff > 180:
        angle_diff -= 360
    # Rotate the robot by the calculated angle difference to face Silvestro
    robot_angle = move_rotation(angle_diff,ser,btsock,offset)
    
finally:
    cv2.destroyAllWindows()
    cam_thread.stop()
