#! /usr/bin/env python

import rospy

#This class calculates the distances of the robot:
# -From obstacles
# -From the goal
#
#It calculates the best direction to move next

class CalculateDirection(x_goal, y_goal):
    def __init__(self, x_goal, y_goal):
	self._x_goal = x_goal
	self._y_goal = y_goal
	self._goal_angle = 360 		#Initialize the angle 
	self._x_axis = 0.0
	self._y_axis = 0.0
	self._distance = 0.0


    #Calculates the distance to the goal from current position
    def distance_to_goal(x_position, y_position):	
	self._x_axis = self._x_goal - x_position
	self._y_axis = self-_y_goal - y_position
	self._distance = round(Math.sqrt( pow(x_axis,2) + pow(y_axis,2) ) , 4) #Number rounded to 4 decimal places	


    #Returns the angle at which the goal is
    def angle_to_goal(x_pos, y_pos):
	distance_to_goal(x_pos, y_pos)		#Calculate distance to the goal
	angle = round(Math.atan2(self._y_axis,self._x_axis),4) #Angle in radians
	degrees = round(angle * (180 / Math.PI) ,4) #Change radians 
	self._goal_angle = degrees
	return self._goal_angle

    #Returns the angle for rotating 
    #Returns the angle at which the robot will have to rotate
    def moving_angle(x_position, y_position):		
	self._goal_angle = angle_to_goal(x_position,y_position)
	#Show the angle the tobot has to rotate:
    	rospy.loginfo('angle: {:.4f}'.format(self._goal_angle))	
        return self._goal_angle
       

    def get_distance()
	return self._distance


