import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import funzionimappa as fmap
import math

altezzafot=67 #misura dal centro fotocamera a terra

class CameraThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)  # Allineamento con il flusso di colore
        self.pipeline.start(self.config)
        self.running = True
        self.color_frame = None
        self.depth_frame = None
        self.lock = threading.Lock()

    def run(self):
        while self.running:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)  # Allinea i frame
            color = aligned_frames.get_color_frame()
            depth = aligned_frames.get_depth_frame()
            if color and depth:
                with self.lock:
                    self.color_frame = np.asanyarray(color.get_data())
                    self.depth_frame = np.asanyarray(depth.get_data())

    def get_frames(self):
        with self.lock:
            return self.color_frame, self.depth_frame

    def stop(self):
        self.running = False
        self.pipeline.stop()

# Funzione per calcolare la distanza con media locale
def calculate_distance(depth_image, cx, cy, kernel_size=5):
    half_k = kernel_size // 2
    # Estrai un'area attorno al centro
    local_area = depth_image[max(0, cy-half_k):cy+half_k+1, max(0, cx-half_k):cx+half_k+1]
    # Filtra i valori di profondità validi
    valid_depths = local_area[local_area > 0]
    if valid_depths.size > 0:
        # Calcola la media in centimetri
        return valid_depths.mean() * 0.1
    return None

def ruota_punto(p1, p2, angolo):
    angolo_attuale=0
    delta_x = p1[0] - p2[0]
    delta_y = p1[1] - p2[1]
    # Calcola la distanza tra p1 e p2
    distanza = math.sqrt(delta_x**2 + delta_y**2)
    # Calcola l'angolo attuale tra p1 e p2 rispetto all'asse x
    angolo_attuale = math.atan2(delta_y, delta_x)
    # Aggiungi l'angolo di rotazione
    nuovo_angolo = angolo_attuale + math.radians(angolo)
    # Calcola le nuove coordinate di p1 dopo la rotazione
    nuovo_x = int(p2[0] + distanza * math.cos(nuovo_angolo))
    nuovo_y = int(p2[1] + distanza * math.sin(nuovo_angolo))
    return nuovo_x, nuovo_y

def dove_object(p0,xo,distanzao,angolo):#calcola posizione oggetto su mappa
    #p0 è robot_pos
    #calcolo 3d->2d come se fosse a nord
    coeff=distanzao/640
    x1= int(p0[0]-(distanzao//2)+xo*coeff)
    y1= int(p0[1]-distanzao)
    pobject=(x1,y1)#doce si trova l'oggetto
    #rotazione rispetto al vero angolo
    angolomap=angolo-270
    if angolomap<0:
        angolomap+=360
    pobject=ruota_punto(pobject,p0,angolomap)
    return pobject

def target(center,distance):
    pixelcm=0
    if center:
        pixelcm=(distance//(1.2))/480
        variaz=(240-center[1])*pixelcm+altezzafot #altezza da terra oggetto
        #print("altezza",variaz)

        pixelcmlarge=distance/640
        pcamer=(190,200)
        pmotor=(200,200)
        x1= int(pcamer[0]-(distance//2)+center[0]*pixelcmlarge)
        y1= int(pcamer[1]-distance)
        pobject=(x1,y1)
        visuale= np.zeros((480,640,3), np.uint8)
        #disegnoun cerchio dove è il motore
        cv2.circle(visuale,pcamer,5,(255,0,255),-1)
        cv2.circle(visuale,pmotor,5,(0,0,255),-1)

        #disegno la linea tra il motore e l'oggetto
        cv2.line(visuale,pmotor,pobject,(255,0,0),1)
        #trovo i gradi tra i due punti 270 è a nord quindi devo modificare rispetto al mio sistema
        angolo=int(fmap.calcola_angolo(pmotor,pobject))
        angolo=angolo-180
        #print("angolo",angolo)
        #print("distanza",distance)
        #cv2.imshow("image",visuale)
        #cv2.waitKey(0)
        return angolo,variaz
    else:
        return -1
