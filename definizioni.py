import numpy as np


# Definizioni

# --- Configurazione della connessione seriale ---
porta = '/dev/ttyUSB0'  # Porta seriale corretta
baudrate = 115200
ser = None

#BT
HC05_MAC = "00:14:03:05:01:C8"  # MAC Address del modulo HC-05
PORT = 1  # Porta RFCOMM standard
#mappa
tile_size = 40
num_tiles = 59
map_width, map_height = 1200, 1200
mappa = np.ones((map_height, map_width, 3), dtype=np.uint8) * 255
obstacle_map = np.ones((num_tiles, num_tiles), dtype=int)  # Nessun ostacolo iniziale


#robot
robot_radius = 20
robot_angle =0
start_pos = [600,600]#[map_width // 2, map_height // 2]
robot_pos = start_pos[:]
robot_pos[0] += tile_size // 2
robot_pos[1] += tile_size // 2
spider=20
distances = [30] * 640  # Distanze dei sensori
robotm=[1] #per capire se grafico o seriale o BT

#colori
# Definire il range per il colore verde
lower_green = np.array([40, 50, 50])
upper_green = np.array([80, 255, 255])
# Definire il range per il colore arancio
lower_orange = np.array([0, 106, 185])
upper_orange = np.array([52, 255, 255])
