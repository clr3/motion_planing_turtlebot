#! /usr/bin/env python

import rospy
import actionlib
from odom_subscriber import OdomTopicReader 
from laser_subscriber import LaserScanReader
from calculate_direction import CalculateDirection
from movement_publisher import MovementPub

class ControlTurtleTest():
    def __init__(self):
        self._goal_distance = 0.0
	self._angle = 0.0
	#Initialize ODOM node:
	self._odom_subscriber_object = OdomTopicReader()
	#Initialize LASER_SCAN node:
	self._laserscan_sub_object = LaserScanReader()
	#Call calculate_direction class
	self._calculate_direction_object = CalculateDirection()
	#Initialize the movement_publisher
	self._movement_pub_object = MovementPub()



    #Logs info of the x position, y position and euler's yaw
    def odom_readings(self):
      x = self._odom_subscriber_object.get_x_position()	
      y = self._odom_subscriber_object.get_y_position()
      yaw = self._odom_subscriber_object.get_eulers_yaw()
      rospy.loginfo('X: {:.4f} + Y: {:.4f}'.format(x,y))	#Shows values recieved for x and y	
      rospy.loginfo('Yaw: {:.4f} '.format(yaw))
	


    #Log
    # No of Objects
    # [Angle(degrees)] -> Distance
    def laserscan_readings(self):
      objs_array = self._laserscan_sub_object.get_objects_array()
      angles_array = self._laserscan_sub_object.get_angles_array()
      length = len(objs_array)

      rospy.loginfo('Objects: {:.4f} '.format(length))
      for x in range(0, length):
	ang = angles_array[x]
	dist = objs_array[x]
	rospy.loginfo('[ {:.2f} ] -> {:.2f} '.format(ang, dist))



    #Log
    # Angle and Magnitude of the resultant forces
    # 
    def calculate_dir(self):
      x = self._odom_subscriber_object.get_x_position()	
      y = self._odom_subscriber_object.get_y_position()
      yaw = self._odom_subscriber_object.get_eulers_yaw()
      objs_array = self._laserscan_sub_object.get_objects_array()
      angles_array = self._laserscan_sub_object.get_angles_array()
   
      calculator = self._calculate_direction_object.forces_without_goal(x, y, yaw, angles_array, objs_array)

      magnitude = self._calculate_direction_object.get_magnitude()
      angle = self._calculate_direction_object.get_angle()

      rospy.loginfo('Magnitude: {:.2f} '.format(magnitude))
      rospy.loginfo('Angle: {:.2f} '.format(angle))
      self._calculate_direction_object.log_forces()

    #Log
    # Angle and Magnitude of the resultant forces
    # 
    def movement_pub(self):
      x = self._odom_subscriber_object.get_x_position()	
      y = self._odom_subscriber_object.get_y_position()
      yaw = self._odom_subscriber_object.get_eulers_yaw()
      objs_array = self._laserscan_sub_object.get_objects_array()
      angles_array = self._laserscan_sub_object.get_angles_array()
   
      calculator = self._calculate_direction_object.forces_without_goal(x, y, yaw, angles_array, objs_array)   

      magnitude = self._calculate_direction_object.get_magnitude()
      angle = self._calculate_direction_object.get_angle()

      self._movement_pub_object.move_robot(angle,magnitude)





if __name__ == "__main__":
    rospy.init_node("turtle_test_node")     #Creating the main node
    myturtletest_object = ControlTurtleTest()	#Object from the main class
    rate = rospy.Rate(4)		#Hz


    ctrl_c = False
    def shutdownhook():
	global ctrl_c
	print "shutdown time!"
    	ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
	
	#Odometry test:    	
	#myturtletest_object.odom_readings()
	#Calculate direction test:    	
	myturtletest_object.calculate_dir()
	#Movement:    	
	#myturtletest_object.movement_pub()
	
    	rate.sleep()


