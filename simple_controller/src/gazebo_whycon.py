#! /usr/bin/python

import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion  
from gazebo_msgs.msg import ModelStates,ModelState
from geometry_msgs.msg import Point,Twist,PoseArray,Pose
from visualization_msgs.msg import Marker
from math import atan2
from std_msgs.msg import String

#x = 0.0
#y = 0.0
#z = 0.0
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
ant_line_point_1 = [0.0,0.0,0.0]
ant_line_point_2 = [0.0,0.0,0.0]




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
	global robot_1_o
	global robot_2_o

	#print("numero de robots: ",len(msg.points))

	#for i in range(len(msg.points)):
	#	globals()['robot_%s' % i] = [0,0,0]
	#	globals()['robot_%s' % i] = msg.points[i]
	#	print ("El robot 0 vale: ", robot_0)

	
	robot_1 = msg.points[0];
	robot_2 = msg.points[1];
	#robot_3 = msg.points[2];
	#robot_4 = msg.points[3];
	#robot_5 = msg.points[4]

	robot_1_o = msg.pose;
	robot_2_o = msg.pose;

	#print ("robot_1_o: ", robot_1_o)

	rot_q = msg.pose.orientation;
	(roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w]);
	



# The node of this script is created.
rospy.init_node ("speed_controller")

# Suscribers and publishers are created too.
sub = rospy.Subscriber("/whycon_ros/markers",Marker,newOdon)

pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

pub1 = rospy.Publisher("/posicion_robot0",Twist,queue_size=2)
pub2 = rospy.Publisher("/posicion_robot1",Twist,queue_size=2)
pub3 = rospy.Publisher("/posicion_robot2",Twist,queue_size=2)
pub4 = rospy.Publisher("/posicion_robot3",Twist,queue_size=2)
pub5 = rospy.Publisher("/posicion_robot4",Twist,queue_size=2)
#pub6 = rospy.Publisher("/posicion_robot5",Twist,queue_size=2)

pub16 = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=10)
pub17 = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=10)
pub18 = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=10)
pub19 = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=10)


pub_line_min_dist = rospy.Publisher('~line_min_dist', Marker, queue_size=1)
pub_line_min_dist_2 = rospy.Publisher('~line_min_dist_2', Marker, queue_size=1)

pub_orientation = rospy.Publisher('robot_1_o', Marker, queue_size=1)
pub_orientation = rospy.Publisher('robot_2_o', Marker, queue_size=1)


# The diferrent objects are created.
msg1 = ModelState();
msg2 = ModelState();
msg3 = ModelState();
msg4 = ModelState();

Path_1 = Marker();
Path_2 = Marker();
Robot_1_o = Marker();
Robot_2_o = Marker();

ant_line_point = Point();
ant_line_point_2 = Point();


# We define the object's name.
msg1.model_name="C1"
msg2.model_name="C2"
msg3.model_name="C3"
msg4.model_name="C4"

speed = Twist()
r = rospy.Rate(20)



while not rospy.is_shutdown():

   # Take position of each robot
	msg1.pose.position = robot_1
	msg2.pose.position = robot_2
	msg3.pose.position = robot_3
	msg4.pose.position = robot_4
	
		
	
   # Path of robot_1

	#print(robot_1.z)
	
	#if robot_1.z < 1.0:
	Path_1.header.frame_id = "/world"	# Is neccesary indicate the place where it will going to show our cases.
	Path_1.type = Path_1.LINE_STRIP		# With this option, it's possible create a marker's line.
	Path_1.action = Path_1.ADD
	Path_1.lifetime = rospy.Duration(10);	# Time that each marker is visible in RVIZ.
	Path_1.id += 1;				# For show marker's line, it's neccesary that each marker has a different ID.			

	Path_1.scale.x = 0.03
	Path_1.scale.y = 0.03
	Path_1.scale.z = 0.03

	Path_1.color.a = 1.0
	Path_1.color.r = 0.0
	Path_1.color.g = 1.0
	Path_1.color.b = 0.0

	Path_1.pose.orientation.x = 0.0
	Path_1.pose.orientation.y = 0.0
	Path_1.pose.orientation.z = 0.0
	Path_1.pose.orientation.w = 1.0

	Path_1.pose.position.x = 0.0
	Path_1.pose.position.y = 0.0
	Path_1.pose.position.z = 0.0

	Path_1.points = []				# A vector with each position is created. This need 2 positions to create a line between both.

	first_line_point = Point()			
	first_line_point = robot_1			# The first point is the real position of the robot.
	Path_1.points.append(first_line_point)		# This position is added to vector of points.

	second_line_point = Point()			
	second_line_point = ant_line_point		# After, the second point is the previous point position of this robot.
	Path_1.points.append(second_line_point)		# Of this form, we're taking the couple of poiints 0-1, 1-2, 2-3, 3-4, etc to create the line.

	
	Robot_1_o.header.frame_id = "/world";
	Robot_1_o.type = Robot_1_o.ARROW
	Robot_1_o.action = Robot_1_o.ADD;
	Robot_1_o.lifetime = rospy.Duration(5);
	Robot_1_o.id = 1;

	Robot_1_o.scale.x = 0.05
	Robot_1_o.scale.y = 0.1
	Robot_1_o.scale.z = 0.0

	Robot_1_o.color.a = 1.0
	Robot_1_o.color.r = 1.0
	Robot_1_o.color.g = 1.0
	Robot_1_o.color.b = 0.0

	#Robot_1_o.pose.position = robot_1

	Robot_1_o.pose.orientation.x = 0.0
	Robot_1_o.pose.orientation.y = 0.0
	Robot_1_o.pose.orientation.z = 0.0
	Robot_1_o.pose.orientation.w = 0.0
	
	Robot_1_o.points = [ant_line_point,first_line_point]
	

	ant_line_point = robot_1			# The last position is saved here.
	print ("robot_1: ", robot_1)

# Path of robot_2
	Path_2.header.frame_id = "/world"	# Is neccesary indicate the place where it will going to show our cases.
	Path_2.type = Path_2.LINE_STRIP		# With this option, it's possible create a marker's line.
	Path_2.action = Path_2.ADD
	Path_2.lifetime = rospy.Duration(10);	# Time that each marker is visible in RVIZ.
	Path_2.id += 1;				# For show marker's line, it's neccesary that each marker has a different ID.			

	Path_2.scale.x = 0.03
	Path_2.scale.y = 0.03
	Path_2.scale.z = 0.03

	Path_2.color.a = 1.0
	Path_2.color.r = 1.0
	Path_2.color.g = 0.0
	Path_2.color.b = 0.0

	Path_2.pose.orientation.x = 0.0
	Path_2.pose.orientation.y = 0.0
	Path_2.pose.orientation.z = 0.0
	Path_2.pose.orientation.w = 1.0

	Path_2.pose.position.x = 0.0
	Path_2.pose.position.y = 0.0
	Path_2.pose.position.z = 0.0

	Path_2.points = []				# A vector with each position is created. This need 2 positions to create a line between both.

	first_line_point_2 = Point()			
	first_line_point_2 = robot_2			# The first point is the real position of the robot.
	Path_2.points.append(first_line_point_2)		# This position is added to vector of points.

	second_line_point_2 = Point()			
	second_line_point_2 = ant_line_point_2		# After, the second point is the previous point position of this robot.
	Path_2.points.append(second_line_point_2)		# Of this form, we're taking the couple of poiints 0-1, 1-2, 2-3, 3-4, etc to create the line.

	Robot_2_o.header.frame_id = "/world";
	Robot_2_o.type = Robot_2_o.ARROW
	Robot_2_o.action = Robot_2_o.ADD;
	Robot_2_o.lifetime = rospy.Duration(5);
	Robot_2_o.id = 1;

	Robot_2_o.scale.x = 0.05
	Robot_2_o.scale.y = 0.1
	Robot_2_o.scale.z = 0.0

	Robot_2_o.color.a = 1.0
	Robot_2_o.color.r = 1.0
	Robot_2_o.color.g = 1.0
	Robot_2_o.color.b = 0.0

	#Robot_2_o.pose.position = robot_1

	Robot_2_o.pose.orientation.x = 0.0
	Robot_2_o.pose.orientation.y = 0.0
	Robot_2_o.pose.orientation.z = 0.0
	Robot_2_o.pose.orientation.w = 0.0
	
	Robot_2_o.points = [ant_line_point,first_line_point]

	ant_line_point_2 = robot_2			# The last position is saved here.

   # Publish data
	# Position of each robot
	pub.publish(speed)
	pub1.publish(robot_1,robot_1)
	pub2.publish(robot_2,robot_2)
	#pub3.publish(robot_3,robot_3)
	#pub4.publish(robot_4,robot_4)
	pub16.publish(msg1)
	pub17.publish(msg2)
	#pub18.publish(msg3)
	#pub19.publish(msg4)

	# Path of each robot
	pub_line_min_dist.publish(Path_1)
	pub_line_min_dist_2.publish(Path_2)

	pub_orientation.publish(Robot_1_o)
	pub_orientation.publish(Robot_2_o)

	# Time of sleep
	r.sleep()
	
	

	

