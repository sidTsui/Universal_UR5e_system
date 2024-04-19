#!/usr/bin/env python3

#source: https://github.com/hsaeidi-uncw/ur5e_control.git, in clas slides, online resources
#Sidney Tsui
#robotics_lab7

import rospy
import math

#outline from ur5e_control.git by hsaeidi-uncw

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import UInt8 #from lecture slides
from robot_vision_lectures.msg import SphereParams #needs sphereparams

#imports for camera to base transformations
import tf2_ros
from tf.transformations import *
import tf2_geometry_msgs

raw_x = 0
raw_y = 0
raw_z = 0
rad = 0

# Adding fields for motion trigger - ACL
plan_printed = False
move = False
params_received = False
#processes data from ROS topic about the sphere

def get_sphere(data):
#global varrs can be changed through the entire node
	global raw_x, raw_y, raw_z, rad
	global params_received
	#assiging values to global varrs 
	raw_x = data.xc
	raw_y = data.yc
	raw_z = data.zc
	rad = data.radius
	params_received = True


def getMove(data):
	global move 
	move = data.data
	
if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	sphere_sub = rospy.Subscriber('/sphere_params', SphereParams, get_sphere)
	move_sub = rospy.Subscriber('move', Bool, getMove)
	plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# define a plan variable
	plan = Plan()
	
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	#source: lect 20, slide 11-12, https://wiki.ros.org/tf2/Tutorials
	while not rospy.is_shutdown():
		if params_received:
			try:
				trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
				#create message
				frame_pt = tf2_geometry_msgs.PointStamped()#init obj
				frame_pt.header.frame_id = 'camera_color_optical_frame'#set ID
				frame_pt.header.stamp = rospy.get_rostime()#timestam
				frame_pt.point.x = raw_x
				frame_pt.point.y = raw_y
				frame_pt.point.z = raw_z
				#point to base frame (point 2)
	   	 		
				new_pt = tfBuffer.transform(frame_pt, 'base', rospy.Duration(1.0))
	   	 		#camera to base
				plan_point1 = Twist()
				point_mode1 = UInt8() #from lecture slides
				# just a quick solution to send two target points
				# define a point close to the initial position
				#initial positions from running "rosrun ur5e_control manual_initialization"
				#copied from terminal
				plan_point1.linear.x = -0.014347799658889583 ##all initial positions 
				plan_point1.linear.y = -0.40875115097293
				plan_point1.linear.z = 0.27437802975997405
				plan_point1.angular.x =  3.126134498268236
				plan_point1.angular.y = 0.016630341936829298
				plan_point1.angular.z = 1.5308106732880025
				point_mode1.data = 0
				# add this point to the plan
				plan.points.append(plan_point1)
				plan.modes.append(point_mode1) #from lecture slides
		
				plan_point2 = Twist()
				point_mode2 = UInt8() #from lecture slides
				# define a point away from the initial position
				#move to position 2
				plan_point2.linear.x = new_pt.point.x
				plan_point2.linear.y = new_pt.point.y
				plan_point2.linear.z = new_pt.point.z + 0.01# decrease z (yaw) to move down for position 2
				plan_point2.angular.x = 3.126134498268236 #angular position stay stagnent because only linear points move
				plan_point2.angular.y = 0.016630341936829298
				plan_point2.angular.z = 1.5308106732880025
				point_mode2.data = 0
				# add this point to the plan
				plan.points.append(plan_point2)
				plan.modes.append(point_mode2) #from lecture slides
				
				plan_pointPU = Twist()
				point_modePU = UInt8() #from lecture slides
				# define a point away from the initial position
				#move to position 2
				plan_pointPU.linear.x = new_pt.point.x
				plan_pointPU.linear.y = new_pt.point.y
				plan_pointPU.linear.z = new_pt.point.z + 0.01# decrease z (yaw) to move down for position 2
				plan_pointPU.angular.x = 3.126134498268236 #angular position stay stagnent because only linear points move
				plan_pointPU.angular.y = 0.016630341936829298
				plan_pointPU.angular.z = 1.5308106732880025
				point_modePU.data = 2
				# add this point to the plan
				plan.points.append(plan_pointPU)
				plan.modes.append(point_modePU) #from lecture slides
				
		
				plan_point3 = Twist()
				point_mode3 = UInt8() #from lecture slides
				# define a point close to the initial position
				#move to position 3
				plan_point3.linear.x =  new_pt.point.x + 0.15 #decrease x (pitch) to move horizontal for position 3
				plan_point3.linear.y = -0.40875115097293
				plan_point3.linear.z = 0.27437802975997405 #revert to initial z position to move back up
				plan_point3.angular.x = 3.126134498268236 #angular positions stay stagnent
				plan_point3.angular.y = 0.016630341936829298
				plan_point3.angular.z = 1.5308106732880025
				point_mode3.data = 0
				# add this point to the plan
				plan.points.append(plan_point3)
				plan.modes.append(point_mode3) #from lecture slides
		
				plan_point4 = Twist()
				point_mode4 = UInt8() #from lecture slides
				# define a point close to the initial position
				#move to position 4
				plan_point4.linear.x =  new_pt.point.x + 0.15 #keep x pitch position 
				plan_point4.linear.y = -0.40875115097293
				plan_point4.linear.z = new_pt.point.z + 0.01 #decrease z(yaw) to same position as point 2 for position 4
				plan_point4.angular.x = 3.126134498268236#angular positions stay stagnent 
				plan_point4.angular.y = 0.016630341936829298
				plan_point4.angular.z = 1.5308106732880025
				point_mode4.data = 0
				# add this point to the plan
				plan.points.append(plan_point4)
				plan.modes.append(point_mode4) #from lecture slides
				
				plan_pointD = Twist()
				point_modeD = UInt8() #from lecture slides
				# define a point close to the initial position
				#move to position 4
				plan_pointD.linear.x =  new_pt.point.x + 0.15 #keep x pitch position 
				plan_pointD.linear.y = -0.40875115097293
				plan_pointD.linear.z = new_pt.point.z + 0.01 #decrease z(yaw) to same position as point 2 for position 4
				plan_pointD.angular.x = 3.126134498268236#angular positions stay stagnent 
				plan_pointD.angular.y = 0.016630341936829298
				plan_pointD.angular.z = 1.5308106732880025
				point_modeD.data = 1
				# add this point to the plan
				plan.points.append(plan_pointD)
				plan.modes.append(point_modeD) #from lecture slides
				
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Frames not available!!!')
				loop_rate.sleep()
				continue
				
			if move:
				# publish the plan
				plan_pub.publish(plan)
			else:
				if not plan_printed:
					print(plan)
					plan_printed = True
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
