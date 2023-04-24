import cv2 as cv

capture = cv.VideoCapture(0)

while True:
	ret, frame = capture.read()

	cv.imshow("frame", frame)
	if cv.waitKey(1) == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()