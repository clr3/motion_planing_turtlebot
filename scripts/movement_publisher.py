#! /usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist, Vector3

#This class recieves information from the calculate_direction class which tells the robot if there is an object on the way and which way to move to prevent a crash.
#
# 0.7 is the max linear velocity that can be given to this turtlebot. 
#

     
class MovementPub():

    def __init__(self):

      self._pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
      self._twist = Twist()
      self._dampener = 0.001
      self.linear=[0.6,0,0]
      self.angular=[0,0,0]
      
	
    #Angle must be in radians: -pi <= angle <= pi
    def move_robot(self, angle, magnitude):

      driving_forward = True 	

      rospy.loginfo('Angle: {:.2f} '.format(angle))
      rospy.loginfo('Magnitude: {:.2f} '.format(magnitude))

	#Calculate the linear velocity depending on the magnitude recived:
      lin_vel = abs(magnitude * self._dampener)
      if lin_vel > 0.7 or lin_vel < -0.7: 
	lin_vel = 0.6
      if lin_vel < 0:			#Make the value always positive
	lin_vel = -(lin_vel)

	#Calculating movement:
      if 100 > magnitude > -100 :		#If there are forces on the robot, stop and turn
        
	driving_forward = False
	self.linear[0]=0

 	if 0 < angle < (math.pi/2):				#At 0 to 90 deg, rotate left.
          rospy.loginfo('0-90 Turn right... ')
     	  self.angular[2]=-0.175					#0.175 rads = 10 de
	
      	if (math.pi /2) < angle < (math.pi):			#At 90 to 180 deg, rotate right.
          rospy.loginfo('90-180 Turn left... ')
     	  self.angular[2]=0.175

      	if (-math.pi/2) > angle > (math.pi):			#At 180 to 270 deg, rotate right.
          rospy.loginfo('180-270 Turn left... ')
     	  self.angular[2]=0.175

      	if 0 > angle > (-math.pi /2):				#At 270 to 360 deg, rotate left.
          rospy.loginfo('270-360 Turn right... ')
      	  self.angular[2]=-0.175
	
      else:					#If there is no object nearby, move straight
        driving_forward = True
        rospy.loginfo('Going Forward... ')
	self.linear[0]=lin_vel
        print 'Linear vel: ' + str(self.linear[0])
      	self.angular[2] = 0
      
      print 'Linear vel: ' + str(self.linear[0])
      print 'Angular vel: ' + str(self.angular[2])

      if driving_forward:
	self.linear[0]=0.6

      #Publish movement:
      self._pub.publish(Twist(Vector3(self.linear[0],self.linear[1],self.linear[2]), Vector3(self.angular[0],self.angular[1],self.angular[2])))
      #self._pub.publish(self._twist)


    def stop_robot(self):
	self._twist.linear.x = 0
     	self._twist.angular.z=0
	self._pub.publish(self._twist)



if __name__ == "__main__":
    
    movement_pub_object = MovementPub()
    rospy.init_node('mov_publisher_node')
    rate = rospy.Rate(10) 
    
    ctrl_c = False

    def shutdownhook():
        global ctrl_c
        rospy.loginfo("shutdown time!")
        ctrl_c = True
        movement_pub_object.stop_robot()
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:

	movement_pub_object.move_robot(0,0)
        rate.sleep()
        
