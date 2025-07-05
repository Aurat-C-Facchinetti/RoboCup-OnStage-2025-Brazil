import pyrealsense2 as rs
import numpy as np
import cv2

# Configura il pipeline della fotocamera
pipeline = rs.pipeline()
config = rs.config()

# Abilita sia il flusso colore che quello di profondità
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Stream di profondità
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # Stream colore

# Avvia lo streaming
pipeline.start(config)

try:
    while True:
        # Attendi un frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Converti in array NumPy
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Normalizza la profondità per visualizzarla meglio
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Mostra entrambi gli stream
        cv2.imshow('RealSense D400 - Colore', color_image)
        cv2.imshow('RealSense D400 - Profondità', depth_colormap)

        # Esci premendo 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Rilascia le risorse
    pipeline.stop()
    cv2.destroyAllWindows()

