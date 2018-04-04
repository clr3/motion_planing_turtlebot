#! /usr/bin/env python

import rospy

#This class calculates the distances of the robot:
# -From obstacles
# -From the goal
#
#It calculates the best direction to move next. Requires infromation from /laser_scan and /odom .
#
# Assuming the values recieved are based on 180 deg view. 
#
# Values obtained from this class must be dampened before applying it to the robot. 
#

class CalculateDirection():
    def __init__(self):
	self._dx = 0.0
	self._dy = 0.0
	self._distance = 0.0
	#These arrays hold values from laserscan:
	self._x_components = []
	self._y_components = []
	self._angles = []

	self._deltax_components = []
	self._deltay_components = []

	self._obj_effect_radius = 5	#Repulsive force doesnt't exist outside it's threshhold
	self._obj_min_distance = 0.8	#Repulsive force will be infinite if the bot gets too close to an object.

	#Variables to hold the final calculation:
	self._X = 0
	self._Y = 0
	self._Magnitude = 0
	self._Angle = 0


    def set_goal(x_goal, y_goal):
	self._goal_angle = 360 		#Initialize the angle 
	self._x_goal = x_goal
	self._y_goal = y_goal
	
	self._deltax_goal = 0
	self._deltay_goal = 0
	
	self._goal_threshhold = 20 	#The attracting force will be grater outside the threshhod
	self._goal_radius = 2



    #Calculates the distance to the goal from current position
    def delta_goal(x_position, y_position):	
	self._deltax_goal = self._x_goal - x_position
	self._deltay_goal = self-_y_goal - y_position


    #Calculates for each object: x_position and y_position + magnitude despending on the distance
    #
    #For each object, the resultant force will be at a 180 deg away.
    #The values returnes indicate the forces given by the objects.
    def calculate_repulsive_forces(x_curr, y_curr, yaw, angles_array[], objects_array[]):    
	self._angles = angles_array		#Contains the angles in degrees

	measeurements = len(angles_array[]) -1 	#Take -1 to make up fot the 0 deg angle reading
	
	for x in range(0, measurements):
	    angle = math.radians(self._angles[x])
	    #Calculate the position of the components relative to initial state of the bot:
	    self._x_components[x] = x_curr + (objects_array[x] * math.cos(yaw + angle))
	    self._y_components[x] = y_curr + (objects_array[x] * math.sin(yaw + angle))

	    if objects_array[x] < self._obj_min_distance:   #If the robot is to close to the object the force will be high.
		self._dx_components[x] = (-200) * (self._x_components[x]) * math.cos(angle)
		self._dy_components[x] = (-200) * math.sin(angle)
	    
	    else if objects_array[x] < self._obj_effect_radius:	#If the robot is inside the theshhold.Set a scaling force
		self._dx_components[x] = (-30) * (self._x_components[x] - self._obj_min_distance) * math.cos(angle)
		self._dy_components[x] = (-30) * (self._y_components[x] - self._obj_min_distance) * math.sin(angle)

	    else		#If the robot is far from the object it wont be affected by it
		self._dx_components[x] = 0
		self._dy_components[x] = 0
	    


    #The bot will recieve "repulsive forces" from objects and move away from them, without a goal in particular.
    def forces_without_goal(x_curr, y_curr, yaw, angles_array[], objects_array[])
      
      calculate_repulsive_forces(x_curr, y_curr, yaw, angles_array[], objects_array[])
      
      for x in range(0, len(self._angles)-1):
	self._X += self._dx_components[x]
	self._Y += self._dy_components[x]

