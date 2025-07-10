import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import funzionimappa as fmap
import math

altezzafot=67 # Height from camera center to the ground in cm

# Thread class to handle RealSense camera streaming
class CameraThread(threading.Thread):
    def __init__(self):
        super().__init__() 
        self.pipeline = rs.pipeline() # Create a RealSense pipeline
        self.config = rs.config() # Configuration for the pipeline
        # Enable color and depth streams at 640x480 resolution and 30 FPS
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)  # Align depth to color frame
        self.pipeline.start(self.config)
        self.running = True
        self.color_frame = None
        self.depth_frame = None
        self.lock = threading.Lock()

    def run(self):
        while self.running: # Thread loop: continuously get frames while running
            frames = self.pipeline.wait_for_frames() # Wait for a new frame set
            aligned_frames = self.align.process(frames) # Align depth to color
            color = aligned_frames.get_color_frame() # Extract color frame
            depth = aligned_frames.get_depth_frame() # Extract depth frame
            # Only proceed if both frames are valid
            if color and depth:
                with self.lock:
                    # Convert frames to arrays
                    self.color_frame = np.asanyarray(color.get_data())
                    self.depth_frame = np.asanyarray(depth.get_data())

    def get_frames(self):
        # Return current frames with thread safety
        with self.lock:
            return self.color_frame, self.depth_frame

    def stop(self):
        # Stop the thread and camera pipeline
        self.running = False
        self.pipeline.stop()

# Function to compute distance using a local average filter around (cx, cy)
def calculate_distance(depth_image, cx, cy, kernel_size=5):
    half_k = kernel_size // 2
    local_area = depth_image[max(0, cy-half_k):cy+half_k+1, max(0, cx-half_k):cx+half_k+1] # Extract a square region around (cx, cy)
    valid_depths = local_area[local_area > 0] # Filter out zero values (invalid depth)
    # Return the average depth in cm if valid values exist
    if valid_depths.size > 0:
        return valid_depths.mean() * 0.1
    return None

# Rotate point p1 around point p2 by a given angle in degrees
def ruota_punto(p1, p2, angolo):
    angolo_attuale=0
    delta_x = p1[0] - p2[0]
    delta_y = p1[1] - p2[1]
    distanza = math.sqrt(delta_x**2 + delta_y**2) # Distance between points
    angolo_attuale = math.atan2(delta_y, delta_x) # Current angle between points

    nuovo_angolo = angolo_attuale + math.radians(angolo) # Add rotation in radians
    
    # Compute new coordinates after rotation
    nuovo_x = int(p2[0] + distanza * math.cos(nuovo_angolo))
    nuovo_y = int(p2[1] + distanza * math.sin(nuovo_angolo))
    return nuovo_x, nuovo_y

# Estimate object position in the map given robot's position and detection parameters
def dove_object(p0,xo,distanzao,angolo):#calcola posizione oggetto su mappa
    # p0: robot position on the map
    coeff=distanzao/640
    # Approximate object position assuming robot is facing north
    x1= int(p0[0]-(distanzao//2)+xo*coeff)
    y1= int(p0[1]-distanzao)
    pobject=(x1,y1)
    # Adjust the object position by rotating it to the actual robot orientation
    angolomap=angolo-270
    if angolomap<0:
        angolomap+=360
    # Rotate the point around the robot to get correct map position
    pobject=ruota_punto(pobject,p0,angolomap)
    return pobject

# Compute the angle and vertical variation of a detected object
def target(center,distance):
    pixelcm=0
    if center:
        # Convert vertical position to real height from ground
        pixelcm=(distance//(1.2))/480
        variaz=(240-center[1])*pixelcm+altezzafot # Object height from ground

        # Compute real-world x/y position relative to the camera
        pixelcmlarge=distance/640 
        pcamer=(190,200) # Camera position in image/map
        pmotor=(200,200) # Motor base position
        x1= int(pcamer[0]-(distance//2)+center[0]*pixelcmlarge)
        y1= int(pcamer[1]-distance)
        pobject=(x1,y1)
        # Visualize the detection
        visuale= np.zeros((480,640,3), np.uint8)
        cv2.circle(visuale,pcamer,5,(255,0,255),-1)
        cv2.circle(visuale,pmotor,5,(0,0,255),-1)
        cv2.line(visuale,pmotor,pobject,(255,0,0),1) # Line from motor to object
        # Compute angle from motor to object; adjust because north is 270°
        angolo=int(fmap.calcola_angolo(pmotor,pobject))
        angolo=angolo-180
        return angolo,variaz # Return angle and object height
    else:
        return -1
