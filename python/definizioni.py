import numpy as np # Library for arrays and mathematical operations

# --- Serial connection configuration ---
porta = '/dev/ttyUSB0'  # Correct serial port for Arduino
baudrate = 115200
ser = None

# --- Bluetooth configuration ---
HC05_MAC = "00:14:03:05:01:C8"  # MAC address of the HC-05 module
PORT = 1  # Standard RFCOMM port

# --- Map setup ---
tile_size = 40
num_tiles = 59
map_width, map_height = 1200, 1200
mappa = np.ones((map_height, map_width, 3), dtype=np.uint8) * 255 # Create an empty (white) map image
obstacle_map = np.ones((num_tiles, num_tiles), dtype=int)  # No obstacles


# --- Robot settings ---
robot_radius = 20
robot_angle =0
start_pos = [600,600] # Initial position (center of the map)
robot_pos = start_pos[:]
robot_pos[0] += tile_size // 2 # Center the robot inside the tile (X axis)
robot_pos[1] += tile_size // 2 # Center the robot inside the tile (Y axis)
spider=20
distances = [30] * 640  # Distanze dei sensori
robotm=[1] # Movement mode 

# --- Color detection ranges (HSV) ---
# Green color range
lower_green = np.array([40, 50, 50])
upper_green = np.array([80, 255, 255])
# Orange color range
lower_orange = np.array([0, 106, 185])
upper_orange = np.array([52, 255, 255])
