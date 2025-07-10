import cv2
import numpy as np
import math
from collections import defaultdict

# --- Initial Settings ---
width, height = 300, 300  # Size of the display window
center = (width // 2, height // 2)  # Center of the window
lengths = [17, 17, 25]  # Length of each robotic arm segment
angles = [0, 0, 0]  # Initial rotation angles of segments
selected_segment = 0  # Index of the segment currently controlled
pos_map = defaultdict(list)  # Dictionary to map endpoints to angle configs
altpav=46 # Distance from arm center to floor
point=[0,0] # Target point

# --- Function to calculate the endpoint of the arm based on angles ---
def calculate_endpoint(center, lengths, angles):
    x, y = center  # Start from the base position
    cumulative_angle = 0  # Initialize the total rotation angle

    for i in range(len(lengths)):  # Iterate through each segment of the arm
        cumulative_angle += angles[i]  # Add the current joint angle to the total
        # Calculate the endpoint of this segment based on the current cumulative angle
        x_new = x + lengths[i] * math.cos(math.radians(cumulative_angle + 90))
        y_new = y + lengths[i] * math.sin(math.radians(cumulative_angle + 90))
        x, y = x_new, y_new # Update the current position to the new endpoint

    return int(x), int(y) # Return the final endpoint of the arm 

# --- Function to generate a map of all reachable positions ---
def generate_position_map(center, lengths):
    pos_map = defaultdict(list)
    for a1 in range(90, 181, 1): # First joint: 90° to 180°
        for a2 in range(-90, 91, 1): # Second joint: -90° to +90°
            for a3 in range(0, 21, 1): # Third joint: 0° to 20°
                angles = [a1, a2, a3]
                endpoint = calculate_endpoint(center, lengths, angles)
                if endpoint[0] < center[0]: # Only consider positions left of center
                    pos_map[endpoint].append(angles) # Save angles for this endpoint
    return pos_map

# --- Function to find the best angle configuration for a given object position ---
def find_closest_configuration(dist_oggetto,alt_oggetto):
    global point
    point=(int(center[0]-dist_oggetto),int(center[1]+altpav-alt_oggetto-12)) # Calculate target point from object's distance and height
    print(point)
    # If exact point exists, return the first configuration
    if point in pos_map:
        return pos_map[point][0]

    # Otherwise, find the closest available point
    closest_point = None
    min_distance = float('inf')
    for p in pos_map.keys():
        distance = math.dist(p, point)
        if distance < min_distance:
            min_distance = distance
            closest_point = p

    # If no close point found or out of reach, return None
    if closest_point is None or min_distance > sum(lengths):
        return None
        
    return pos_map[closest_point][0]

# --- Function to draw the arm segments on a window ---
def draw_segments(angles):
    global lengths,point
    image = np.zeros((height, width, 3), dtype=np.uint8)
    # Calculate segment endpoints and draw them
    points = [center]
    for i, length in enumerate(lengths):
        angle = sum(angles[:i+1])  # Sum the cumulative angle untill the current segment
        x1 = points[-1][0] + length * math.cos(math.radians(angle + 90))  
        y1 = points[-1][1] + length * math.sin(math.radians(angle + 90))  
        points.append((int(x1), int(y1)))
        cv2.line(image, points[-2], points[-1], (255, 0, 0), 2) # Drawing a blue lines for segments
    
    # Draw a horizontal line as the floor
    pavstart=(center[0]-20,center[1]+altpav)
    pavend=(center[0]+20,center[1]+altpav)
    cv2.line(image, pavstart,pavend , (255, 255, 255), 2)
    # Draw a green dot at the target point
    cv2.circle(image, point, 2 , (0, 255, 0), 2)
    cv2.imshow("Segmenti Rotanti", image)
    cv2.waitKey(1)


# --- Precompute reachable positions map ---
print("genero posizioni braccio")
pos_map = generate_position_map(center, lengths)
