import cv2
import numpy as np


vidcap = cv2.VideoCapture(0)
vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)

success,image = vidcap.read()

count = 0
success = True

while success:
	name = "./catkin_ws/src/zed_recoder1/"+str(count)+"R.jpg"
	print name

	success,image = vidcap.read()
	
	pts1 = np.float32([[0,0],[1280,0],[0,720],[1280,720]])	
	pts2 = np.float32([[1280,720],[0,720],[1280,0],[0,0]])
	M = cv2.getPerspectiveTransform(pts1,pts2)
	dst = cv2.warpPerspective(image,M,(1280,720))



	cv2.imshow("image",dst)
	cv2.imwrite(name,dst)
	cv2.waitKey(5)
	count+=1



