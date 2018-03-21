#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 

#This class recieves information from calculate_direction which tells the robot 
# in which direction to move!
#

class MovementPub(object):

    def __init__(self):

    	rospy.init_node('mov_publisher_node')

        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=11)

        self._var = Twist()

	self._vel_dampener = 0.01
        self._max_linearspeed = 0.7
	self._min_speed = 0.1
	self._max_angularspeed = 3.14
	self._angularspeed = 1.0
	        
	

    def move_robot(self, angle, force):
      	self.angular_speed()
	self.linear_speed(force)
	rospy.loginfo('LinearSpeed: {:.2f} '.format(self._var.linear.x))
	rospy.loginfo('AngularSpeed: {:.2f} '.format(self._var.angular.z))	

	self._pub.publish(self._var)
	rospy.spin()
	



    #The linear speed of the robot will be different depending on the distance from the goal and other objects
    def linear_speed(self, force):
	velocity = force * self._vel_dampener
	
	#Check and change the values:
	if force == 0:
	    velocity = 0	
	elif velocity > self._max_linearspeed:
	    velocity = self._max_linearspeed
	elif velocity < self._min_speed:
	    velocity = self._min_speed

	#Speed will be publisher with  move_robot() 
	self._var.linear.x = velocity




    #To rotate the robot to the direction in which it has to move 
    #Negative to the left, positive to the right
    def angular_speed(self):
	velocity = self._angularspeed

	if 0 < angle and angle < 1.5: #If the angle smaller than 90deg, turn right	
	    velocity = -velocity
	elif angle > 1.5:	#If the angle is bigger than 90deg, turn left
            velocity = velocity
	else:
	    velocity = 0

	self._var.angular.z = velocity
					
		
		

    def stop_robot(self):
	self._var.linear.x = 0.0
	self._var.angular.z = 0.0
	self._pub.publish(self._var)



 



if __name__ == "__main__":

    angle = raw_input('Angle(rads): ')
    force = raw_input('Force: ')
    movement_pub_object = MovementPub()
    rate = rospy.Rate(5)
    
    ctrl_c = False

    def shutdownhook():
    	global ctrl_c
    	global twist_object
    	global pub
       
     	movement_pub_object.stop_robot()
    	rospy.loginfo("shutdown time!")
    	ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
	
    	movement_pub_object.move_robot(float(angle),float(force))
    	rate.sleep()
        
