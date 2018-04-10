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

arr_size = 0
_scanner_array = []
#This array will hold values for every 10 deg:
#Degrees are measured from left to right, clockwise
angles_array = [0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180]
# number of angles we want :	
_no_of_measurements = 19
objects_array = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


class LaserScanReader():

    def __init__(self, topic_name = '/scan'):	
      	self._topic_name = topic_name
      	self._sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback) 
    	

    def laser_callback(self, msg):	#Recieves LaserScan Data
      global arr_size
      global _scanner_array
      #Make a copy of laser array:
      _scanner_array = msg.ranges
      #Check range of laser array:
      arr_size = len(_scanner_array)
      self.create_objects_array()
      return 
	

	#Will create an array wich contains laser scan values at various degrees
    def create_objects_array(self):
      global arr_size
      global _scanner_array
      global _no_of_measurements

      read_now = 0		#Initialize the angles count
      tick = arr_size / (_no_of_measurements-1)
      count = 0

      for x in range(0, arr_size+1):
	if(x == read_now):
	  obj = _scanner_array[x]
	  objects_array[count] = obj 
	  count += 1 

	  if read_now == 40:	
	    read_now = 39
          read_now += tick


    def get_angles_array(self):
      global angles_array
      ang = angles_array
      return ang


    def get_objects_array(self):
      global objects_array
      objs = objects_array
      return objs



if __name__ == "__main__":
   
    laserscan_sub_object = LaserScanReader()	

    rospy.init_node('test_laserscan_sub_node')		#Creating the node 
    rate = rospy.Rate(0.2)

    ctrl_c = False
    def shutdownhook():
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:			
	
	objs_array = laserscan_sub_object.get_objects_array()
	angle_array = laserscan_sub_object.get_angles_array()
	length = len(angle_array)

        rospy.loginfo('Objects: {:.4f} '.format(length))
   	for x in range(0, length):
	  ang = angle_array[x]
	  dist = objs_array[x]
	  rospy.loginfo('[ {:.2f} ] -> {:.2f} '.format(ang, dist))

        rate.sleep()
