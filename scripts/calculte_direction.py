#! /usr/bin/env python

import rospy

#This class calculates the distances of the robot:
# -From obstacles
# -From the goal
#
# It calculates the best direction to move next
#
# Assuming the values recieved are based on 180 deg view
#
# Values optained from this class must be dampened before applying it to the robot
#

class CalculateDirection():
    def __init__(self, x_goal, y_goal):

	self._x_goal = x_goal
	self._y_goal = y_goal	
	self._goal_angle = 0.0

	self._deltax_goal = 0
	self._deltay_goal = 0
	#The attracting force will be grater outside the threshho
	self._goal_threshhold = 10 		
	self._goal_radius = 2

	self._x_components = []
	self._y_components = []
	self._forces_angle = []

	self._deltax_components = []
	self._deltay_components = []
	#If an object is this close, the repelling force will be bigger
	self._objs_threshold = 5
	self._obj_min_distance = 0.8

	self._X = 0
	self._Y = 0
	self._FORCE = 0 
	self._ANGLE = 0
	


    def calculate_Force(x_curr, y_curr, yaw, laser_scan[])
	
	#First calculate all the components for the diferent foces:
	self.attracting_force(x_curr, y_curr)
	self.repelling_forces(x_curr, y_curr, yaw, laser_scan[])
	
	#Add up all the forces:
	self._X = self._deltax_goal
	self._Y = self._deltay_goal

	comps = len(self._deltax_components)
	for x (0, comps)
	    self._X += self._deltax_components[x]
	    self._Y += self._deltay_components[x]
	


    del force()
	self._FORCE = round(math.sqrt(math.pow(self._X,2) + math.pow(self._Y, 2)),4)
	return self._FORCE


    del force_angle()
	
	return self._ANGLE
	


    #Calculates for the goal: distance from current position, angle and final force from current position
    def attracting_force(x_curr, y_curr):	
	
	self._deltax_goal = self._x_goal - x_curr
	self._deltay_goal = self-_y_goal - y_curr

	distance = round(Math.sqrt( pow(x_axis,2) + pow(y_axis,2) ) , 4) 	#Distance to the goal
	self._goal_angle = round(Math.atan2(self._y_axis,self._x_axis),4) 	#Angle in radians
	
	if distance < self._goal_radius:		#If the robot is IN the goal, the force will be zero
	    self._x_goal = self._y_goal = 0
	else if distance < self._goal_threshold 	#If the robot is within the affected area set high attraction
	    #Multiply by a big number to get the attraction
	    self._deltax_goal = 300 * (distance - self._goal_radius) * 	math.cos(self._goal_angle)		 
	    self._deltay_goal = 300 * (distance - self._goal_radius) * 	math.sin(self._goal_angle)
	else 						#If the robot is to far from the goal, set force very high:
	    self._deltax_goal = 500 * (distance - self._goal_radius) * 	math.cos(self._goal_angle)		 
	    self._deltay_goal = 500 * (distance - self._goal_radius) * 	math.sin(self._goal_angle)



    #Calculates for each object: Angle, x_position and y_position
    #
    def repelling_forces(x_curr, y_curr, yaw, laser_scan[]):    
	
	measeurements = len(laser_scan[]) -1 	#Take -1 to make up fot the 0 angle reading
	angle_change = 180 / measurements
	angle = 0

	x_tot = 0.0
	y_tot = 0.0

	for x (0, measurements)
	    self._forces_angle[x] = math.radians(angle)
	    #Calculate the position of the components:
	    self._x_components[x] = x_curr + (laser_scan[x] * math.cos(yaw + self._forces_angle[x]))
	    self._y_components[x] = y_curr + (laser_scan[x] * math.sin(yaw + self._forces_angle[x]))

	    if laser_scan[x] < self._obj_min_distance:   #If the robot is to close to the object the force will be higher.
		self._deltax_components[x] = (-500) * math.cos(self._forces_angle[x])
		self._deltay_components[x] = (-500) * math.sin(self._forces_angle[x])
	    
	    else if laser_scan[x] < self._objs_threshhold:	#If the robot is inside the theshold.Set a scaling force
		self._deltax_components[x] = (-30) * (self._x_components[x] - self._obj_min_distance) * math.cos(self._forces_angle[x])
		self._deltay_components[x] = (-30) * (self._x_components[x] - self._obj_min_distance) * math.sin(self._forces_angle[x])

	    else		#If the robot is far from the object it wont be affected by it
		self._deltax_components[x] = 0
		self._deltay_components[x] = 0





