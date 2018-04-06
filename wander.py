#! /usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan



#This class recieves the values from /odom topic
#We are intrested in locations x and y , and Euler's Yaw
class OdomTopicReader():
    def __init__(self, topic_name = '/odom'):	#Initialization

        #rospy.init_node('odom_topic_sub_node')	#Creating the node
    
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




#This class recieves data from the scanner
# Angles are given in radians by sensor_msgs.LaserScan
# 
# For the robot in gazebo:
# angle_min: -1.57079994678
# angle_max: 1.57079994678
# angle_increment: 0.00436940183863.
# range_max = 30.0
#
_arr_size = 0
_scanner_array = []
class LaserScanReader():
    def __init__(self, topic_name = '/scan'):		

      #rospy.init_node('laserscan_sub_node')		#Creating the node 
      self._objects_array = [] 	#Empty array to hold the objects
      
      self._topic_name = topic_name
      self._sub = rospy.Subscriber(self._topic_name, LaserScan, self.laser_callback) 

      #WThis array will hold values for every 10 deg:
      #Degrees are measured from the right, anticlockwise
      self._angles_array = [0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180]
      # number of angles we want -1:	
      self._no_of_measurements = 18	
      
    def set_goal(x_goal, y_goal):        
      self._threshold = 1		
      self._x_goal = x_goal
      self._y_goal = y_goal

    def laser_callback(data):
	global _scanner_array
	global _arr_size
	#Make a copy of laser array:
	_scanner_array = data.ranges
	#Check range of laser array:
	_arr_size = len(self._scanner_array)
	print("Arr Size: " + str(self._arr_size))

	read_now = 0		#Initialize the angles count
    	tick = _arr_size / self._no_of_measurements

        print("Tick: " + str(tick))
        print("Arr Size: " + str(_arr_size))

        for x in range(0, self._no_of_measurements):
	  print("Measurements: " + str(self._no_of_measurements))
	  print("X: " + str(x))
	  if(x == read_now):
	    self._objects_array.append(_scanner_array[x])
  	    read_now += tick
 	return 
	

    def get_angles_array(self):
      return self._angles_array

    def get_objects_array(self):
      #self.create_objects_array()
      return self._objects_array




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
    def calculate_repulsive_forces(x_curr, y_curr, yaw, angles_array, objects_array):    
	self._angles = angles_array		#Contains the angles in degrees

	measeurements = len(angles_array) -1 	#Take -1 to make up fot the 0 deg angle reading
	
	for x in range(0, measurements):
	    angle = math.radians(self._angles[x])
	    #Calculate the position of the components relative to initial state of the bot:
	    self._x_components[x] = x_curr + (objects_array[x] * math.cos(yaw + angle))
	    self._y_components[x] = y_curr + (objects_array[x] * math.sin(yaw + angle))

	    if objects_array[x] < self._obj_min_distance:   #If the robot is to close to the object the force will be high.
		self._dx_components[x] = (-200) * (self._x_components[x]) * math.cos(angle)
		self._dy_components[x] = (-200) * math.sin(angle)
	    
	    elif objects_array[x] < self._obj_effect_radius:	#If the robot is inside the theshhold.Set a scaling force
		self._dx_components[x] = (-30) * (self._x_components[x] - self._obj_min_distance) * math.cos(angle)
		self._dy_components[x] = (-30) * (self._y_components[x] - self._obj_min_distance) * math.sin(angle)

	    else:		#If the robot is far from the object it wont be affected by it
		self._dx_components[x] = 0
		self._dy_components[x] = 0


    #The bot will recieve "repulsive forces" from objects and move away from them, without a goal in particular.
    def forces_without_goal(x_curr, y_curr, yaw, angles_array, objects_array):
      
      calculate_repulsive_forces(x_curr, y_curr, yaw, angles_array, objects_array)
      
      for x in range(0, len(self._angles)-1):
	self._X += self._dx_components[x]
	self._Y += self._dy_components[x]

      self._Angle = math.atan2(self._X, self._Y)		#angle in radians

      mag = 0 	#Initialize
      #Find the cuadrant of the force to find it's magnitude
      if 0 < self._Angle < (math.pi /2):			#Between 0 and 90 deg:
	mag = self._Y / math.sin(self._Angle)
      if (math.pi /2) < self._Angle < (math.pi):		#Between 90 and 180 deg:
        mag = self._Y / math.sin(math.pi - self._Angle)
      if (-math.pi/2) > self._Angle > (math.pi):		#Between 180 and 270 deg:
	mag = self._Y / math.sin(self._Angle)
      if 0 > self._Angle > (-math.pi /2):			#Between 270 and 360 deg:
	mag = self._Y / math.sin(-self._Angle)

      self._Magnitude = mag
	
      rospy.loginfo("Angle: " + self._Angle) 
      rospy.loginfo("Magnitude: " + self._Magnitude) 


    def get_magnitude(self):
      return self._Magnitude
	
    def get_angle(self):
      return self._Angle




#This class recieves information from the calculate_direction class which tells the robot if there is an object on the way and
# which way to move to prevent a crash.
#
# 0.7 is the max linear velocity that can be given to this turtlebor. 
class MovementPub(object):

    def __init__(self):
      #rospy.init_node('mov_publisher_node')

      self._pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
      self._twist = Twist()
      self._linear=[0.7,0,0]
      self._angular=[0,0,0]
      self._driving_forward = True 	
      self._dampener = 0.01
        
	
    #Angle must be in radians: -pi <= angle <= pi
    def move_robot(self, angle, magnitude):
      
	#Calculate the linear velocity depending on the magnitude recived:
      lin_vel = magnitude * self._dampener
      if lin_vel > 0.7: 
	lin_vel = 0.7
      if lin_vel < -0.7:
	lin_vel = -0.7

	#Calculating movement:
      if magnitude == 0 or angle == 0:		#If there is no force on the robot, keep moving forward
        self._driving_forward = True
      else:					#If there is an object nearby
	self._driving_forward = False

 	if 0 < angle < (math.pi /2):				#At 0 to 90 deg, rotate right.
	  self._linear=[lin_vel,0,0]
     	  self._angular=[0,0,0.5]
	
      	if (math.pi /2) < angle < (math.pi):			#At 90 to 180 deg, rotate left.
          self._linear=[lin_vel,0,0]
     	  self._angular=[0,0,-0.5]

      	if (-math.pi/2) > self._Aangle > (math.pi):		#At 180 to 270 deg, rotate left.
          self._linear=[lin_vel,0,0]
     	  self._angular=[0,0,-0.5]

      	if 0 > angle > (-math.pi /2):				#At 270 to 360 deg, rotate right.
          self._linear=[lin_vel,0,0]
     	  self._angular=[0,0,0.5]
      	
	#Check if driving forward:
      if self._driving_forward:
	self._linear=[0.7,0,0]
     	self._angular=[0,0,0]
      
	#Publish movement:
      self._pub.publish(Twist(Vector3(linear[0],linear[1],linear[2]), Vector3(angular[0],angular[1],angular[2])))


    def stop_robot(self):
	self.twist.linear.x = 0.0
	self.twist.angular.z = 0.0
	self._pub.publish(self.twist)




#Connects all the classes 
#
# Robot will move around a map avoiding obstacles, without a specific goal.

class ControlTurtle():
    def __init__(self):
        
	#Initialize all nodes:
	self.init_odom_subscriber()  
	self.init_calculate_direction()
	self.init_laser_subscriber()
	self.init_movement_publisher()   

        #Create an object of the odom_subscriber
    def init_odom_subscriber(self):		
        self._odom_subscriber_object = OdomTopicReader()
    
	#Create an object of the calculate_direction 
    def init_calculate_direction(self):
        self._calculate_direction_object = CalculateDirection()
    
	#Create an object of the laser_subscriber 
    def init_laser_subscriber(self):
        self._laser_subscriber_object = LaserScanReader()

	#Create an object of the laser_subscriber
    def init_movement_publisher(self):
	self._move_robot = MovementPub()


	#Conects laserscan and odom readings
    def move_robot(self):
     	
	#Get the position of the robot:
	x_position = self._odom_subscriber_object.get_x_position() 
	y_position = self._odom_subscriber_object.get_y_position()
	yaw = self._odom_subscriber_object.get_eulers_yaw()
	#Get the angles and objects arrays:
	angles = self._laser_subscriber_object.get_angles_array()
	objects = self._laser_subscriber_object.get_objects_array()

	#Now, calculate the angle and magnitude for the movement
	self._calculate_direction.forces_without_goal(x_position, y_position, yaw, angles, objects)
	moving_angle = self._calculate_direction.get_angle()
	moving_mag = self._calculate_direction.get_magnitude()

	#make the robot move:
	self._move_robot.move_robot(self._move_robot, moving_angle, moving_mag)






if __name__ == "__main__":
    rospy.init_node("wander")		#Creating the main node
    controlturtle_object = ControlTurtle()	#Object from the main class
    rate = rospy.Rate(1)


    ctrl_c = False
    def shutdownhook():
	global ctrl_c
	print "shutdown time!"
    	ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
	#Move the robot:    	
	controlturtle_object.move_robot()	
    	rate.sleep()
