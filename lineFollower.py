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
    roi = image[300:400,0:640]
    
    # Crear la máscara para detectar la línea negra
    blackLine = cv2.inRange(image, black_lower, black_upper)
    greenSignal = cv2.inRange(image, (0,65,0),(100,200,100))
    
    
    kernel = np.ones((3,3),np.uint8)
    blackLine = cv2.erode(blackLine, kernel, iterations=2)
    blackLine = cv2.dilate(blackLine, kernel, iterations=9)
    
    greenSignal = cv2.erode(greenSignal, kernel, iterations=2)
    greenSignal = cv2.dilate(greenSignal, kernel, iterations=9)
    
    # Encontrar los contornos en la máscara
    contours, _ = cv2.findContours(blackLine.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contoursGreen, _ = cv2.findContours(greenSignal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Dibujar el contorno verde en la región de interés
    
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	
	image = frame.array	
	Blackline = cv2.inRange(image, (0,0,0), (75,75,75))	
	kernel = np.ones((3,3), np.uint8)
	Blackline = cv2.erode(Blackline, kernel, iterations=5)
	Blackline = cv2.dilate(Blackline, kernel, iterations=9)	
	img_blk,contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	contours_blk_len = len(contours_blk)
	if contours_blk_len > 0 :
	 if contours_blk_len == 1 :
	  blackbox = cv2.minAreaRect(contours_blk[0])
	 else:
	   canditates=[]
	   off_bottom = 0	   
	   for con_num in range(contours_blk_len):		
		blackbox = cv2.minAreaRect(contours_blk[con_num])
		(x_min, y_min), (w_min, h_min), ang = blackbox		
		box = cv2.boxPoints(blackbox)
		(x_box,y_box) = box[0]
		if y_box > 358 :		 
		 off_bottom += 1
		canditates.append((y_box,con_num,x_min,y_min))		
	   canditates = sorted(canditates)
	   if off_bottom > 1:	    
		canditates_off_bottom=[]
		for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
		   (y_highest,con_highest,x_min, y_min) = canditates[con_num]		
		   total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
		   canditates_off_bottom.append((total_distance,con_highest))
		canditates_off_bottom = sorted(canditates_off_bottom)         
		(total_distance,con_highest) = canditates_off_bottom[0]         
		blackbox = cv2.minAreaRect(contours_blk[con_highest])	   
	   else:		
		(y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]		
		blackbox = cv2.minAreaRect(contours_blk[con_highest])	 
	 (x_min, y_min), (w_min, h_min), ang = blackbox
	 x_last = x_min
	 y_last = y_min
	 if ang < -45 :
	  ang = 90 + ang
	 if w_min < h_min and ang > 0:	  
	  ang = (90-ang)*-1
	 if w_min > h_min and ang < 0:
	  ang = 90 + ang	  
	 setpoint = 320
	 error = int(x_min - setpoint) 
	 ang = int(ang)	 
	 box = cv2.boxPoints(blackbox)
	 box = np.int0(box)
	 cv2.drawContours(image,[box],0,(0,0,255),3)	 
	 cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
	 cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
	 cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
        
    
        
    
    # Mostrar la imagen
    cv2.imshow("Video", image)
    cv2.imshow("Vide3o", greenSignal)
    cv2.imshow("VideoN", blackLine)
    
    
    
    # Esperar a que se presione la tecla 'q' para salir
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    # Limpiar el buffer de la cámara para el siguiente fotograma
    rawCapture.truncate(0)

# Detener la cámara y cerrar las ventanas
camera.close()
cv2.destroyAllWindows()
