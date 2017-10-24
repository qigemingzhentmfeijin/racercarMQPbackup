import rospy
import math
import numpy as np 
import cvxopt
from cvxopt import matrix
from cvxopt.blas import dot 
from cvxopt.solvers import qp 
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray




def initValues():


	global mass
	global gamma

	global delta_soft
	global ep

	global vd
	global v0
	global x2
	global z
	global cd
	global ca

	global m
	global p_sc
	global pcc
	global f0
	global f1
	global f2
	global g
	global Cf
	global Cr
	global R
	global rps2mps
	global a
	global b
	global Cr
	global Iz

	global p2
	global yaw
	global y
	global vy
	global psi
	global r
	global x_pos
	

	a = 1.11
	b = 1.59
	Iz = 2315.3				#moment of 
	p2 = 1000.0				#relax factor
	yaw = 0.0

	vy = 0.0
	psi = 0.0
	psi_dot = 0
	r = 0


	

	
	rospy.init_node('Lk')
	
	timeStamp = 0.1
	mass = 5.0
	gamma = 0.0001

	delta_soft = 0.00002
	ep = 10.0


	vd = 15.0					#desire speed
	v0 = 20.0					#leading car speed
	x2 = 0.0 					#starting speed(follower)
	z = 10.0					#initial distance to leading car
	#54
	cd = 0.8					#
	ca = 0.2

				#
	m = mass
	p_sc = 10.0**5
	pcc = 10.0**10
	f0 = 0.1
	f1 = 5.0
	f2 = 0.25
	g = 9.81

	Cf = 133000					#resistance for front wheel and back
	Cr = 98800

	rps2mps = 0.1109
	R = 100000.0
	x_pos = 0.0
	
	

def runLk():
	global timeStamp 
	global mass
	global gamma

	global delta_soft
	global ep

	global vd
	global v0
	global x2
	global z
	global cd
	global ca

	global m
	global p_sc
	global pcc
	global f0
	global f1
	global f2
	global g
	global Cf
	global Cr	
	global R
	global psi_dot
	global a
	global b
	global Cr
	global Iz

	global p2
	global yaw

	global vy
	global psi
	global r			#?
	global x_pos
	global pubAngle
	




	vx = x2+0.0001
	d = x2/(R+0.0000001) 					#desire yaw rate

	LgV = 2.0*(psi_dot-d)*a*Cf/Iz
	LfV = 2.0*(psi_dot-d)*((b*Cr-a*Cf)*vy/(vx*Iz)-(a**2*Cf+b**2*Cr)*r/(vx*Iz))+ep*(psi_dot-d)**2

	A_clf_lk = matrix([0.0,LgV,0.0,1.0,0.0])
	b_clf_lk = -1.0*LfV
	A_fcbf_lk = matrix([0.0,1.0,0.0,0.0,0.0])
	b_fcbf_lk = 1.0

	A_lk = matrix([[A_clf_lk],[0.0,0.0,0.0,(x_pos)**2, 0.0]])
	b_lk = matrix([[b_clf_lk],[1.0]])

	Fr = f0-f1*x2-f2*(x2**2)
	fx = matrix([[x2],[-1.0*Fr/m]])
	gx = matrix([[0.0],[1.0/m]])

	A_clf = matrix([2.0*(x2-vd)/m,0.0,-1.0,0.0,0.0])
	b_clf = 2.0*((x2-vd)*Fr/m)-ep*((x2-vd)**2)

	hl_hard = (-1.8*x2+z)

	b_cbf = (1.8*Fr+m*(v0-x2))/(m*(1+hl_hard)*hl_hard)+gamma/(-1.0*np.log(hl_hard/(1+hl_hard)))

	A_cbf = matrix([(1.2/(m*(1+hl_hard)*hl_hard)), 0.0, 0.0,0.0, 0.0])

	A_fcbf = matrix([[1.0,2.0,2.0,2.0,-1.0],[-1.0,0.0,0.0,0.0,-1.0]])
	b_fcbf = matrix([[ca*m*g],[cd*m*g]])

	A_acc = matrix([[A_clf],[A_cbf],[A_fcbf]])
	b_acc = matrix([[b_cbf],[b_cbf],[b_fcbf]])

	H = matrix([[2.0/(m**2),0.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0,0.0],[0.0,0.0,p_sc,0.0,0.0],[0.0,0.0,0.0,p2,0.0],[0.0,0.0,0.0,0.0,pcc]]).T

	A = matrix([[A_lk],[A_acc]]).T
	B = matrix([[b_lk],[b_acc]]).T
	F = matrix([[-2.0*Fr/(m**2)],[0.0],[0.0],[0.0],[0.0]]).T
	cvxopt.solvers.options['show_progress']=False
	U = cvxopt.solvers.qp(H,F,A,B)

	#print U["x"][0], U["x"][1], U["x"][2], U["x"][3], U["x"][4]
#/mg
	#speed = speed+U["x"][0]/0.1/m/g

	angle = calAngle(U["x"][1])
	print angle
	pubAngle.publish(angle)

def calAngle(degree):
	mid = 10350

	if(degree<-0.52):
		return 6653
	if(degree>0.52):


		return 13007
	return (degree)/0.52*2757+10350

def updateR(data):
	global R
	global psi_dot
	global x_pos
	R = data.data[0]
	psi_dot = R
	x_pos = data.data[1]
	runLk()

def updateSpeed(data):
	global x2
	global rps2mps
	x2 = data.data*rps2mps
	x2 = 100

def initMessages():
	global pubAngle
	speedSub = rospy.Subscriber("/racer/teensy/rpm", Float32, updateSpeed)
	rSub = rospy.Subscriber("/racer/LaneFollower/yawOffset",Float64MultiArray, updateR)
	pubAngle = rospy.Publisher("/racer/teensy/steer", Int32, queue_size = 1)

if __name__== '__main__':
	initValues()
	initMessages()

	rospy.spin()
	
