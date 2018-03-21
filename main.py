#! /usr/bin/env python

import rospy
import actionlib
import sys
#from std_srvs.srv import Trigger, TriggerRequest
#from .msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
#from cmd_vel_pub import CmdVelPub
#from scripts import OdomTopicReader
#import calculate_direction
#import laser_subscriber
#import movement_publisher




#Connects all the classes 
#
#Takes the goal position as an imput from the terminal

class ControlTurtle():
    def __init__(self, x_gol, y_gol):
        self._goal_distance = 0.0
	self._angle = 0.0
	#Initialize all nodes:
	self.init_odom_subscriber()  
	self.init_calculate_direction()
	self.init_laser_subscriber()
	self.init_movement_publisher()
	self._x_goal = x_gol
	self._y_goal = y_gol    

        #Create an object of the odom_subscriber
    def init_odom_subscriber(self):		
        self._odom_subscriber_object = OdomTopicReader()
    
	#Create an object of the calculate_direction 
    def init_calculate_direction(self):
        self._calculate_direction_object = CalculateDirection(self._x_goal, self._y_goal)
    
	#Create an object of the laser_subscriber 
    def init_laser_subscriber(self):
        self._laser_subscriber_object = LaserScanReader(self._x_goal, self._y_goal)

	#Create an object of the laser_subscriber
    def init_movement_publisher(self):
	self._move_robot = MovementPub()

	#Conects laserscan and odom readings
    def move(self):
	#Get the position of the robot:
	x_position = self._odom_subscriber_object.get_x_position() 
	y_position = self._odom_subscriber_object.get_y_position()
       	yaw = self._odom_subscriber_object.get_yaw_euler()			#Euleur's 'yaw'
	laser_scan = self._laser_subscriber_object.create_angles_array()	#Array containing the objs
	#Find the force and angle:
	self._calculate_direction_object.calculate_Force(x_position, y_position, yaw, laser_scan)
	force = self._calculate_direction_object.force()
	angle = self._calculate_direction_object.angle()	
	#make the robot move:
	self._move_robot.move_robot(self._move_robot, angle, force)


if __name__ == "__main__":
    rospy.init_node("turtle_main_node")		#Creating the main node
    x = raw_input('X direction: ')
    y = raw_input('Y direction: ')
    controlturtle_object = ControlTurtle(x,y)	#Object from the main class
    rate = rospy.Rate(5)


    ctrl_c = False
    def shutdownhook():
	global ctrl_c
	print "shutdown time!"
    	ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
	#Show the distance to the goal:
    	rospy.loginfo('distance: {:.4f}'.format(self._calculate_direction_object.get_distance()))
	
	#Move the robot:    	
	controlturtle_object.move()	
    	rate.sleep()

