#! /usr/bin/python

import rospy
import numpy as np
#from PointArray import PointArray
from tf.transformations import euler_from_quaternion  
from gazebo_msgs.msg import ModelStates,ModelState
from geometry_msgs.msg import Point,Twist,PoseArray
from visualization_msgs.msg import Marker
from math import atan2
from std_msgs.msg import String

x = 0.0
y = 0.0
z = 0.0
theta = 0.0
#cont = 0
robot = np.array([0.0,0.0,0.0], dtype=float)
robotaux={}
robot_1 = [0.0,0.0,0.0]
robot_2 = [0.0,0.0,0.0]
robot_3 = [0.0,0.0,0.0]
robot_4 = [0.0,0.0,0.0]
robot_5 = [0.0,0.0,0.0]
robot_6 = [0.0,0.0,0.0]
robot_7 = [0.0,0.0,0.0]
robot_8 = [0.0,0.0,0.0]
robot_9 = [0.0,0.0,0.0]
robot_10 = [0.0,0.0,0.0]
robot_11 = [0.0,0.0,0.0]
robot_12 = [0.0,0.0,0.0]
robot_13 = [0.0,0.0,0.0]
robot_14 = [0.0,0.0,0.0]
robot_15 = [0.0,0.0,0.0]




def newOdon(msg):
	global x
	global y
	global z
	global theta
	global cont
	global robot_
	global robot_1
	global robot_2
	global robot_3
	global robot_4
	global robot_5
	global robot_6
	global robot_7
	global robot_8
	global robot_9
	global robot_10
	global robot_11
	global robot_12
	global robot_13
	global robot_14
	global robot_15

	print("numero de robots: ",len(msg.points))

	#for i in range(len(msg.points)):
	#	globals()['robot_%s' % i] = [0,0,0]
	#	globals()['robot_%s' % i] = msg.points[i]
	#	print ("El robot 0 vale: ", robot_0)
		
		
		
		
	#	robot=[0.0,0.0,0.0]
	#	robot1=[0.0,0.0,0.0]
	
	robot_1 = msg.points[0]
	robot_2 = msg.points[1]
	robot_3 = msg.points[2]
	robot_4 = msg.points[3]
	#robot_5 = msg.points[4]
	#robot_6 = msg.points[5]
	#robot_7 = msg.points[6]
	#robot_8 = msg.points[7]
	#robot_9 = msg.points[8]
	#robot_10 = msg.points[9]
	#robot_11 = msg.points[10]
	#robot_12 = msg.points[11]
	#robot_13 = msg.points[12]
	#robot_14 = msg.points[13]
	#robot_15 = msg.points[14]
	
	
	rot_q = msg.pose.orientation
	#print("rot es ",rot_q)
	(roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	

rospy.init_node ("speed_controller")

#sub = rospy.Subscriber("/whycon/visualization_marker",Marker,newOdon)
sub = rospy.Subscriber("/whycon/visualization_marker",Marker,newOdon)
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
pub1 = rospy.Publisher("/posicion_robot0",Twist,queue_size=2)
pub2 = rospy.Publisher("/posicion_robot1",Twist,queue_size=2)
pub3 = rospy.Publisher("/posicion_robot2",Twist,queue_size=2)
pub4 = rospy.Publisher("/posicion_robot3",Twist,queue_size=2)
pub5 = rospy.Publisher("/posicion_robot4",Twist,queue_size=2)
pub6 = rospy.Publisher("/posicion_robot5",Twist,queue_size=2)
pub7 = rospy.Publisher("/posicion_robot6",Twist,queue_size=2)
pub8 = rospy.Publisher("/posicion_robot7",Twist,queue_size=2)
pub9 = rospy.Publisher("/posicion_robot8",Twist,queue_size=2)
pub10 = rospy.Publisher("/posicion_robot9",Twist,queue_size=2)
pub11 = rospy.Publisher("/posicion_robot10",Twist,queue_size=2)
pub12 = rospy.Publisher("/posicion_robot11",Twist,queue_size=2)
pub13 = rospy.Publisher("/posicion_robot12",Twist,queue_size=2)
pub14 = rospy.Publisher("/posicion_robot13",Twist,queue_size=2)
pub15 = rospy.Publisher("/posicion_robot14",Twist,queue_size=2)
pub16 = rospy.Publisher("/ModelStatesRobot",ModelState,queue_size=10)

msg = ModelState();
	
msg.model_name="coche_0_0"
msg.pose.position.x = 1.0
msg.pose.position.y = 1.0



speed = Twist()
r = rospy.Rate(4)

goal = Point ()
goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
#	inc_x = goal.x - x
#	inc_y = goal.y - y

#	angle_to_goal = atan2(inc_y, inc_x)

#	if abs(angle_to_goal - theta) > 0.1:
#		speed.linear.x = 0.0
#		speed.linear.z = 0.3
#	else:
#		speed.linear.x = 0.5
#		speed.linear.z = 0.0

	
	

	pub.publish(speed)
	#pub1.publish(robot_0,robot_0)
	pub1.publish(robot_1,robot_1)
	pub2.publish(robot_2,robot_2)
	pub3.publish(robot_3,robot_3)
	pub4.publish(robot_4,robot_4)
	pub16.publish(msg)
	r.sleep()
