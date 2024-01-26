import cv2

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L) # video capture source camera (Here webcam of laptop) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

ret,frame = cap.read() # return a single frame in variable `frame`

if ret:
	cv2.imwrite('test_image.png', frame)
else:
	print('Image unable to be captured')

cap.release()
