#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

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
	self._yaw = 0.0
    
    def callback(self, msg):		#Recieves the data from Odometry
	self._x_position = msg.pose.pose.position.x
	self._y_position = msg.pose.pose.position.y
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	self._yaw = euler[2]

        #rospy.loginfo('x: {}, y: {}', format(self._x_position, self._y_position))	

    def get_x_position(self):			#Returns current values of x position
        return self._x_position

    def get_y_position(self):			#Returns current values of y position
        return self._y_position

    def get_eulers_yaw(self):		#Returns current values of w orientation
        return self._yaw





if __name__ == "__main__":
    odom_reader_object = OdomTopicReader()	#Creating object of the OdomTopicReader()
    rospy.init_node('odom_topic_sub_node')	#Creating the node if this class is run as main
    rate = rospy.Rate(0.2)			

    ctrl_c = False
    def shutdownhook():			#This will shut down the program at the end
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    

    while not ctrl_c:				#while this node is running this happens:
        pos_x = odom_reader_object.get_x_position()
	pos_y = odom_reader_object.get_y_position()
	yaw = odom_reader_object.get_eulers_yaw()

        rospy.loginfo('X: {:.4f} + Y: {:.4f}'.format(pos_x, pos_y))	#Shows values recieved for x and y	
	rospy.loginfo('Yaw: {:.4f} '.format(yaw))
		
        rate.sleep()


