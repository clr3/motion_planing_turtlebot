#! /usr/bin/env python

import rospy

#from std_srvs.srv import Trigger, TriggerRequest
#from .msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
#from cmd_vel_pub import CmdVelPub
#import odom_subrcriber.py 
#import calculate_direction.py
#import laser_subscriber.py
#import movement_publisher.py

#Connects all the classes 
#
#

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
        self._calculate_direction_object = CalculateDirection(object, self._x_goal, self._y_goal)
    
	#Create an object of the laser_subscriber 
    def init_laser_subscriber(self):
        self._laser_subscriber_object = LaserScanReader(object, self._x_goal, self._y_goal)

	#Create an object of the laser_subscriber
    def init_movement_publisher(self):
	self._move_robot = MovementPub()

	#Conects laserscan and odom readings
    def move_robot(self):
	#Get the position of the robot:
	x_position = self._odom_subscriber_object.get_x_position() 
	y_position = self._odom_subscriber_object.get_y_position()
	#Find the angle:
	self._angle = self._calculate_direction_object.moving_angle(x_position, y_position)
	#make the robot move:
	self._move_robot.move_robot(self._move_robot, self._angle)


if __name__ == "__main__":
    rospy.init_node("turtle_main_node")		#Creating the main node
    controlturtle_object = ControlTurtle(1.0,10.0)	#Object from the main class
    rate = rospy.Rate(1)


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
	controlturtle_object.move_robot()	
    	rate.sleep()

