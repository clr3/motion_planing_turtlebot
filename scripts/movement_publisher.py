#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 

#This class recieves information from LaserScanSubscriber which tells the robot 
# in which direction to move
#

class MovementPub(object):

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.var = Twist()
        self.linearspeed = 1
        self.angularspeed = 3
        
	
    def move_robot(self, angle):
        if angle < 10 and angle > - 10:
	    self.var.linear.x = self.linearspeed
	else:
 	    self.var.angular.z = self.angularspeed
        self.pub.publish(self.var)
        
	#The linear speed of the robot will be different depending on the distance from the goal and other objects
    def linear_speed(self, speed):
	if speed == 'slow':	
	elif speed == 'med':
	elif speed == 'fast':				
				
			

    del angular_speed(self, speed):

    def stop_robot(self):
	self.var.linear.x = 0.0
	self.var.angular.z = 0.0
	self.pub.publish(self.var)


if __name__ == "__main__":
    rospy.init_node('mov_publisher_node')
    movement_pub_object = MovementPub()
    rate = rospy.Rate(0.5)
    
    
    ctrl_c = False

    def shutdownhook():
        global ctrl_c
        global twist_object
        global pub
        
        rospy.loginfo("shutdown time!")
        
        ctrl_c = True
        movement_pub_object.stop_robot()
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        
        movement_pub_object.move_robot(0.0)
        rate.sleep()
        
