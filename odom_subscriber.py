#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_quaternion

#This class recieves the values from /odom topic
#We are intrested in locations x and y 
#

class OdomTopicReader():
    def __init__(self, topic_name = '/odom'):	#Initialization

	rospy.init_node('odom_topic_sub_node')	#Creating the node

	self._topic_name = topic_name		
	self._sub = rospy.Subscriber(self._topic_name, Odometry, self.callback) #Creating the subscriber
	#Initializing variables:
	self._x_position = 0.0
	self._y_position = 0.0
	self._yaw = 0.0
    
    def callback(self, data):		#Recieves the data from Odometry
	self._x_position = data.pose.pose.position.x
	self._y_position = data.pose.pose.position.y

	quaternion = (
		data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)

	euler = euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	self._yaw = euler[2]
	

        #rospy.loginfo('x: {}, y: {}', format(self._x_position, self._y_position))	

    def get_x_position(self):			#Returns current values of x position
        return self._x_position

    def get_y_position(self):			#Returns current values of y position
        return self._y_position

    def get_yaw_euler(self):
	return self._yaw

    def shutdownhook():			#This will shut down the program at the end
    	global ctrl_c
   	print "shutdown time!"
   	ctrl_c = True


if __name__ == "__main__":

    odom_reader_object = OdomTopicReader()	#Creating object of the OdomTopicReader()
    rate = rospy.Rate(0.5)			

    ctrl_c = False


    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:				#while this node is running this happens:
  	 pos_x = odom_reader_object.get_x_position()
 	 pos_y = odom_reader_object.get_y_position()
   	 rospy.loginfo('x: {:.4f} + y: {:.4f}'.format(pos_x, pos_y))	#Shows values recieved for x and y	
   	 rospy.loginfo('yaw: {:.4f} '.format(yaw))	#Shows values recieved for yaw	
   	 rate.sleep()

