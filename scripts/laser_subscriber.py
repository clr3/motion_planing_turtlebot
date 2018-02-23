#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

#This class recieves data from the scanner
# Angles are given in radians by sensor_msgs.LaserScan
# 
# For the robot in gazebo:
# angle_min: -1.57079994678
# angle_max: 1.57079994678
# angle_increment: 0.00436940183863.
# range_max = 30.0
#

class LaserScanReader():
    def __init__(self, x_goal, y_goal, topic_name = '/scan'):		#'/scan' is the topic that publishes LaserScan
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, LaserScan, self.topic_callback()) # !!!topic_callback needs an argument (msg)   
	self._threshold = 1		
	self._x_goal = x_goal
	self._y_goal = y_goal
	#This will be a copy of LaserScan.ranges[]
	self._scanner_array = []	
	#This array will hold values for every 10 deg:
	#Degrees are measured from the right, anticlockwise
	self._anlges_array = [0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180]	
	#Initialize empty array that will hold the readings from laserscan at each angle:
	self._angles = []		
	
					
        
    def topic_callback(self, msg):
	#Make a copy of laser array:
	self._scanner_arrary = msg.ranges
	#Check range of laser array:
	self._arr_size = len(self._scanner_array)
 	return 
	
	#Will create an array wich containt laser scan values at various degrees
    def create_angles_array(self):
	to

    def get_angles(self):
	create_angles_array()
	return self._angles

    def shutdownhook():
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True





if __name__ == "__main__":
    rospy.init_node('laserscan_sub_node')		#Creating the node
    laserscan_sub_object = LaserScanReader(0.0,0.0)	#Initialize with the values for the goal
    rospy.loginfo(laserscan_sub_object.topic_callback())  #Log 
    rate = rospy.Rate(0.5)

    ctrl_c = False

    rospy.on_shutdown(laserscan_sub_object.shutdownhook())
    
    while not ctrl_c:				
        data = laserscan_sub_object.moving_direction()
        rospy.loginfo(data)
        rate.sleep()
