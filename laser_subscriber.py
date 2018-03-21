#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

#This class recieves data from the scanner
# 
#Angles are given in radians by sensor_msgs.LaserScan
# 
# To change the amount of readings just edit: self._angles_array
#
# For the robot in gazebo:
# angle_min: -1.57079994678
# angle_max: 1.57079994678
# angle_increment: 0.00436940183863.
# range_max = 30.0
#

class LaserScanReader():
    def __init__(self, topic_name = '/scan'):		#'/scan' is the topic that publisher LaserScan
	
	rospy.init_node('laserscan_sub_node')		#Creating the node   
     
	self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, LaserScan, queue_size=10)
	
	self._scan = LaserScan()
	self._scanner_arrary = []
    	self._arr_size
	
	#This array will hold values for every 10 deg:
	#Degrees are measured from the right, anticlockwise
	self._angles_array = [0,30,60,90,120,150,180]	
	self._scans
					
    def get_readings(self):
	#Make a copy of laser array:
	self._scanner_arrary = self._scan.ranges
	#Check range of laser array:
	self._arr_size = len(self._scanner_array)

    #Will return an array wich contains laser scan values at various degrees (this will be the distance from the object
    def create_angles_array(self):

	#How many angles will be checked depending in the _angles_array length:
	measuerements = len(self._angles_array)	-1 	#Take -1 because of the angle at 0 degrees

	read = self._arra_size / measurements 
	read_now = 0

	for x in range(0, measuerements):
	    self._scans = [self.scanner_array[read_now]]
	    read_now += read

	return self._scans



    def shutdownhook():
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True


if __name__ == "__main__":

    laserscan_sub_object = LaserScanReader()	

    rospy.loginfo('LaserScan: {:.4f} '.format(laser_scan))
    rate = rospy.Rate(0.5)

    ctrl_c = False

    rospy.on_shutdown(laserscan_sub_object.shutdownhook())
    
    while not ctrl_c:				
    	data = laserscan_sub_object.moving_direction()
    	rospy.loginfo(data)
    	rate.sleep()
