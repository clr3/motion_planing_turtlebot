#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

#This class recieves data from the scanner
# 
# 

class LaserScanReader():
    def __init__(self, x_goal, y_goal):
        self._topic_name = '/kobuki/laser/scan'
        self._sub = rospy.Subscriber(self._topic_name, LaserScan, self.topic_callback)        
	self._threshold = 1		
	self._x_goal = x_goal
	self._y_goal = y_goal
        
    def topic_callback(self, msg):
	#Check range of laser array:
	
	#Record values for every angle 
 	return 
	
    def shutdownhook():
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True





if __name__ == "__main__":
    rospy.init_node('laserscan_sub_node')		#Creating the node
    laserscan_sub_object = LaserScanReader()
    rospy.loginfo(laserscan_sub_object.topic_callback())  #Log 
    rate = rospy.Rate(0.5)

    ctrl_c = False

    rospy.on_shutdown(laserscan_sub_object.shutdownhook())
    
    while not ctrl_c:				
        data = laserscan_sub_object.moving_direction()
        rospy.loginfo(data)
        rate.sleep()
