import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# Configuración de la cámara
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))

# Esperar a que la cámara arranque
time.sleep(0.1)

# Definir los límites de color para detectar la línea negra
black_lower = np.array([0, 0, 0], dtype=np.uint8)
black_upper = np.array([180, 255, 50], dtype=np.uint8)

# Configurar la región de interés
roi_top = 240
roi_bottom = 320

# Iniciar el bucle principal
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Obtener el array de la imagen
    image = frame.array
    
    # Definir la región de interés
    roi = image[roi_top:roi_bottom, :]
    
    # Convertir la imagen a espacio de color HSV
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Crear la máscara para detectar la línea negra
    mask = cv2.inRange(hsv, black_lower, black_upper)
    
    # Encontrar los contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Dibujar el contorno verde en la región de interés
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        cv2.drawContours(roi, [c], -1, (0, 255, 0), 2)
    
    # Mostrar la imagen
    cv2.imshow("Video", image)
    
    # Esperar a que se presione la tecla 'q' para salir
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    # Limpiar el buffer de la cámara para el siguiente fotograma
    rawCapture.truncate(0)

# Detener la cámara y cerrar las ventanas
camera.close()
cv2.destroyAllWindows()
