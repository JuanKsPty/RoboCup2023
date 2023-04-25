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


# Iniciar el bucle principal
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Obtener el array de la imagen
    image = frame.array
    
    # Definir la región de interés
    roi = image[300:400,0:639]
    
    # Convertir la imagen a espacio de color HSV
    
     
    # Crear la máscara para detectar la línea negra
    blackLine = cv2.inRange(roi, black_lower, black_upper)
    
    greenSignal = cv2.inRange(image, (0,65,0),(100,200,100))
    greenSignal1 = cv2.inRange(image, (0,65,0),(100,200,100))
    
    kernel = np.ones((3,3),np.uint8)
    blackLine = cv2.erode(blackLine, kernel, iterations=2)
    blackLine = cv2.dilate(blackLine, kernel, iterations=9)
    
    greenSignal = cv2.erode(greenSignal, kernel, iterations=2)
    greenSignal = cv2.dilate(greenSignal, kernel, iterations=9)
    
    # Encontrar los contornos en la máscara
    contours, _ = cv2.findContours(blackLine.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contoursGreen, _ = cv2.findContours(greenSignal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Dibujar el contorno verde en la región de interés
    #cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
    if len(contoursGreen) > 0:
        xGr,yGr,wGr,hGr = cv2.boundingRect(contoursGreen[0])
        centerGr = xGr+(wGr/2)
        #cv2.rectangle(roi,(xGr,yGr),(xGr+wGr,yGr+h),(0,0,255),3)
        cv2.line(image, (int(centerGr),0), (int(centerGr),480),(255,0,255),3)
    if len(contours) > 0:
        x,y,w,h = cv2.boundingRect(contours[0])
        center = x+(w/2)
        #cv2.rectangle(roi,(x,y),(x+w,y+h),(0,0,255),3)
        cv2.line(image, (int(center),0), (int(center),480),(255,0,0),3)
    
        
    
    # Mostrar la imagen
    cv2.imshow("Video", image)
    cv2.imshow("Vide3o", greenSignal1)
    
    
    # Esperar a que se presione la tecla 'q' para salir
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    # Limpiar el buffer de la cámara para el siguiente fotograma
    rawCapture.truncate(0)

# Detener la cámara y cerrar las ventanas
camera.close()
cv2.destroyAllWindows()
