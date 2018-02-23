#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

#This class recieves the values from /odom topic
#We are intrested in locations x and y 
#

class OdomTopicReader():
    def __init__(self, topic_name = '/odom'):	#Initialization
	self._topic_name = topic_name		
	self._sub = rospy.Subscriber(self._topic_name, Odometry, self.callback) #Creating the subscriber
	#Initializing variables:
	self._x_position = 0.0
	self._y_position = 0.0
	self._w_orientation = 0.0
	self._z_orientation = 0.0
    
    def callback(self, msg):		#Recieves the data from Odometry
	self._x_position = msg.pose.pose.position.x
	self._y_position = msg.pose.pose.position.y
	self._w_position = msg.pose.pose.orientation.w
	self._z_position = msg.pose.pose.orientation.z

        #rospy.loginfo('x: {}, y: {}', format(self._x_position, self._y_position))	

    def get_x_position(self):			#Returns current values of x position
        return self._x_position

    def get_y_position(self):			#Returns current values of y position
        return self._y_position

    def get_w_orientation(self):		#Returns current values of w orientation
        return self._w_orientation

    def get_z_orientation(self):		#Returns current values of w orientation
        return self._z_orientation





if __name__ == "__main__":
    rospy.init_node('odom_topic_sub_node')	#Creating the node
    odom_reader_object = OdomTopicReader()	#Creating object of the OdomTopicReader()
    rate = rospy.Rate(0.5)			

    ctrl_c = False
    def shutdownhook():			#This will shut down the program at the end
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    

    while not ctrl_c:				#while this node is running this happens:
        pos_x = odom_reader_object.get_x_position()
	pos_y = odom_reader_object.get_y_position()
	or_w = odom_reader_object.get_w_orientation()
	or_z = odom_reader_object.get_z_orientation()
        rospy.loginfo('x: {:.4f} + y: {:.4f}'.format(pos_x, pos_y))	#Shows values recieved for x and y	
	
	#rospy.loginfo('x: {:.4f} + y: {:.4f}'.format(or_w, or_z))	#Shows values recieved for w and z	
        rate.sleep()

