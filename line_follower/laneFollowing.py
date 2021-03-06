import rospy
import math
import cv2
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray






def calLanePos(img):
	img = img[510:720,0:1280]

	hist = []
	for c in range(0,1280):
		counter = 0
		for r in range(0,200):
			counter+=img[r,c]

		hist.append(counter)
	#=========================================================
	midpoint = 640
	leftStart = np.argmax(hist[:midpoint])
	rightStart = np.argmax(hist[midpoint:])+midpoint
	print leftStart, rightStart
	#number of sliding window
	nwindows = 10
	#hight of window
	height = np.int(200/nwindows)
	nonzero = img.nonzero()
	nonzeroY = np.array(nonzero[0])
	nonzeroX = np.array(nonzero[1])
	#print nonzero[0], nonzero[1]

	leftCurrent = leftStart
	rightCurrent = rightStart
	
	margin = 100
	minpix = 50
	
	leftShift = []
	rightShift = []
	image = np.dstack((img,img,img))
	leftPoints = []
	rightPoints = []
	for w in range(nwindows):
		yTop = 210-(w+1)*height
		yBot = 210-w*height
		
		xLeft1 = leftCurrent-margin
		xLeft2 = leftCurrent+margin
		xRight1 = rightCurrent-margin
		xRight2 = rightCurrent+margin

		cv2.rectangle(image,(xLeft1,yTop),(xLeft2,yBot),(0,255,0),2)
		cv2.rectangle(image,(xRight1,yTop),(xRight2,yBot),(0,255,0),2)
		#print xLeft1, xLeft2   

		leftDiv = ((nonzeroY >= yTop)&(nonzeroY < yBot)&(nonzeroX >= xLeft1)&(nonzeroX <= xLeft2))
		rightDiv = ((nonzeroY >= yTop)&(nonzeroY < yBot)&(nonzeroX >= xRight1)&(nonzeroX <= xRight2))
		#print leftDiv,rightDiv
		leftShift.append(leftDiv)
		rightShift.append(rightDiv)
		
		#print len(leftDiv), len(rightDiv)
		if len(nonzeroX[leftDiv]) > minpix:
			leftCurrent = np.int(np.mean(nonzeroX[leftDiv]))
		if len(nonzeroX[rightDiv]) > minpix:        
			rightCurrent = np.int(np.mean(nonzeroX[rightDiv]))
		leftPoints.append(leftCurrent)
		rightPoints.append(rightCurrent)
	
	#cv2.imshow("image11",image)

	#print leftPoints, rightPoints

	#=========================================================
	#print len(leftPoints), len(rightPoints)

	yaw,offset = findYaw(leftPoints,rightPoints)
	print yaw,offset

	return yaw,offset



def findYaw(left, right):
	offset = 0.0
	R = 0.0
	L = 0.0

	if(right[0]-left[0]>400):		
		L = calYaw(left)
		R = calYaw(right)
		offset = ((right[0]-left[0])/2.0+left[0]-640)/400.0
		return (L+R)/2.0,offset
	offset = (300-(right[0]-640))/400	
	return R,offset
	#M = calYaw(mid)




def calYaw(points):
	r = []
	l = 60.0/40.0
	#print len(points)
	for i in range(1,4):
		
		a1 = points[i]+0.0
		a2 = points[i+3]+0.0
		a3 = points[i+6]+0.0
		P1 = (a1-a2)/400.0
		P2 = (a2-a3)/400.0
		rtemp = (P1+P2)/2.0+2.0*(l**2)/(P1+P2+0.00001)
		r.append(rtemp)
		#print rtemp
	return (r[0]+r[1]+r[2])/3.0



def publisher(yaw,offset):
	global pub
	result = Float64MultiArray()
	result.data = []
	result.data.append(yaw)
	result.data.append(offset)
	pub.publish(result)



if __name__ =='__main__':
	global pub
	rospy.init_node('gymFollow')
	pub = rospy.Publisher("/racer/LaneFollower/yawOffset",Float64MultiArray, queue_size = 10)

	vidcap = cv2.VideoCapture(0)
	vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
	vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)


	count = 0
	success = True

	success,image = vidcap.read()
	rows,cols,ch = image.shape


	count = 0
	success = True
	print success
	while success:

		success,image = vidcap.read()
		rows,cols,ch = image.shape
		pts1 = np.float32([[0,0],[1280,0],[0,720],[1280,720]])	
		pts2 = np.float32([[1280,720],[0,720],[1280,0],[0,0]])
		M = cv2.getPerspectiveTransform(pts1,pts2)
		image = cv2.warpPerspective(image,M,(1280,720))

		pts1 = np.float32([[330,280],[950,280],[0,350],[1280,350]])	
		pts2 = np.float32([[0,0],[1280,0],[0,720],[1280,720]])
		#pts1 = np.float32([[280,330],[280,950],[350,0],[350,1280]])	
		#pts2 = np.float32([[0,0],[0,1280],[720,0],[720,1280]])
	




		M = cv2.getPerspectiveTransform(pts1,pts2)	 
		dst = cv2.warpPerspective(image,M,(1280,720))
		dst = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		medBlur = cv2.medianBlur(dst,5)
		ret,th = cv2.threshold(medBlur,100,255,cv2.THRESH_BINARY)

		#cv2.imshow("1",image)
		#cv2.imshow("2",th)
		#cv2.imshow("3",medBlur)

		#cv2.waitKey(0)
		yaw,offset = calLanePos(th)
		publisher(yaw,offset)
		#cv2.imshow("image1", image)
		print "success"
	
		count+=1

	
	



