import cv2 as cv

image = cv.VideoCapture(0)

while True:
    isTrue, frame = image.read()
   
    cv.imshow('video',frame)

    if cv.waitKey(20) & 0xFF==ord('d'):
        break

image.release()