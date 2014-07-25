#!/usr/bin/env python

"""mover.py: Sends movement commands to the turtlebot"""

__authors__ = ["Nathan Smith", "Dexter Duckworth", "Alex Lalejini"]
__maintainer__ = "Dexter Duckworth"
__date__ = "10/19/13"

"""
Receives message type: 
Sends message type: Twist
"""

"""
Twist message components:
linear (m/s), angular (rad/s)
linear.x - translational velocity along the x-axis = Forward/Backward
angular.x - rotational velocity about the x-axis
angular.y - rotational velocity about the y-axis
angular.z - rotational velocity about the z-axis
http://answers.ros.org/question/9941/twist-message-coordinate-system-convention/
"""

from threading import Thread
import threading
import roslib; roslib.load_manifest("turtlebot_code")
import rospy
from turtlebot_code.msg import Move_Cmd, Converted_Odom
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import robot_intent_utilities as riu
import math
import signal
import sys

#Global variable used to keep track of the turtlebot's current position
move_state = {"roll":0, "pitch":0, "yaw":0, "twist": None, 'x':0, 'y':0, 'z':0}
estop_flag = False
command_flag = False
current_command = "null"



#TODO: Potential problem with not locking critical variables on read in condition!
# - Fix: use local variables to store data held in move_state; lock on update of local variable

class ActionThread(Thread):
	#pass in data we want thread to have access to
	def __init__(self):
		#initialize variables we want access to
		self.cmd = None
		self.param = None
		Thread.__init__(self)
	#run function is executed when instance of ActionThread is spawned
	def run(self):
		#call function passed on load
		self._target(*self._args)

	def load(self, target, param):
		self._target = target
		self._args = (param,)

	
class Mover(object):
	executing_action = False
	ready_message_interval = 5
	
	
	#stop_msg = Twist(angular.z = 0, linear.x = 0) is illegal python code.  Arguments cannot be expressions!
	stop_msg = Twist()
	stop_msg.angular.x = 0
	stop_msg.angular.y = 0
	stop_msg.angular.z = 0
	stop_msg.linear.x = 0
	stop_msg.linear.y = 0
	stop_msg.linear.z = 0

	def __init__(self):
		#Tell ROS about mover
		rospy.init_node('mover', anonymous = False)
		#Subscribe to the commands and odometry topics
		rospy.Subscriber('/robot_intent/commands', Move_Cmd, self.callback_queue)
		rospy.Subscriber('/robot_intent/odometry', Converted_Odom, self.callback_odom)
		rospy.Subscriber('/robot_intent/tracking', String, self.callback_tracking)
		#Twist publisher
		self.publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist)
		self.status_pub = rospy.Publisher("/robot_intent/mover_status", String)
		self.action_thread = ActionThread()
		self.estop_lock = threading.Lock() #lock used to lock estop variable
		self.move_state_lock = threading.Lock() #lock used to lock global move_state variable
		self.angle_lock = False
		self.zeroed_angle = None
		self.correction = riu.no_correction
		self.ignored_commands = ["stop"]
		#Signal setup to send periodic ready messages
		signal.signal(signal.SIGALRM, self.sig_handler)
		signal.alarm(Mover.ready_message_interval)

	def run(self):
		global command_flag, estop_flag, current_command
		while not rospy.is_shutdown():
			#if an estop is sent while the mover is doing nothing, reset it
			if (not Mover.executing_action and estop_flag):
				with estop_lock:
					estop_flag = False
			#check for new command
			if (command_flag):
				#reset the new command flag
				command_flag = False
				#process/run the current command
				self.process_command(current_command)

	def sig_handler(self, signum, frame):
		if not self.action_thread.is_alive():
			self.status_pub.publish(String("ready"))
			signal.alarm(Mover.ready_message_interval)

	def callback_queue(self, data):
		"""Starts the move process"""
		global command_flag, current_command
		#grab action command
		action = data.action
		#Discards pending messages and null/invalid messages
		if action.split(' ')[0] in riu.valid_cmds and not data.pending:
			if action in riu.interrupts:
				self.process_interrupt(action)
			elif action.split(' ')[0] not in self.ignored_commands:
				print("Setting action")
				command_flag = True
				current_command = action

	def callback_odom(self, data):
		"""Updates the movement status dictionary to the turtlebot's current position and velocity"""
		global move_state
		#acquire a lock on global move_state
		with self.move_state_lock:
			move_state['roll'] = data.roll
			move_state['pitch'] = data.pitch
			move_state['yaw'] = data.yaw
			move_state['twist'] = data.twist

			move_state['x'] = data.position.x
			move_state['y'] = data.position.y
			move_state['z'] = data.position.z
			
			if self.zeroed_angle is None:
				self.zeroed_angle = (math.degrees(data.yaw) + 360) % 360

	def callback_tracking(self, data):
		self.correction = data.data

	def process_interrupt(self, interrupt):
		global estop_flag

		if interrupt == 'estop' and Mover.executing_action:
			#acquire a lock on estop_flag variable
			with self.estop_lock:
				estop_flag = True
		elif interrupt == "resume":
			#if resume interrupt, flip the estop_flag off
			print ("RESUME")
			with self.estop_lock:
				estop_flag = False

	def process_command(self, command):
		"""Checks if action_thread is already running; if not, spawn a new thread to handle command."""
		if not Mover.executing_action:
			cmd = command.split(' ')[0]
			try:
				param = float(command.split(' ')[1])
			except:
				param = None
			finally:
				Mover.executing_action = True
				#Load sets the thread's run target and parameters
				self.action_thread.load(getattr(self, cmd), param)
				#spawn an action thread
				self.action_thread.run()
				Mover.executing_action = False

	def left(self, param):
		"""If param == 0, sets turn angle to default value.
		Converts current position angle from radians to degrees.
		Converts negative angles to positive. COntinues to turn left until the 
		current distance to the goal is greater than the previous distance, meaning that the 
		goal has been passed."""
		global estop_flag, move_state
		#If input angle is zero, set angle to default
		if param:
			angle = param
		else:
			angle = riu.default_angle

		signal.alarm(0) #Disable timer interrupt for the duration of the movement
		#safely grab current yaw
		with self.move_state_lock:
			current_yaw = (math.degrees(move_state['yaw']) + 360) % 360
		#Set goal to yaw+angle. Add 360 then mod to account for negative angles but avoid going over 360
		goal = (current_yaw + angle) % 360
		half_goal = (current_yaw + angle/2) % 360
		if self.angle_lock:
			if goal >= 315 and goal < 45:
				goal = self.zeroed_angle
			elif goal >= 45 and goal < 135:
				goal = self.zeroed_angle + 90
			elif goal >= 135 and goal < 225:
				goal = self.zeroed_angle + 180
			elif goal >= 225 and goal < 315:
				goal = self.zeroed_angle + 270
		goal = goal % 360
		half_goal = (current_yaw + angle/2) % 360
		halfway_flag = False #used to flag if we've already sent out a halfway message
		#Anonymous function that calculates the current counterclockwise distance to the goal
		chkdist = lambda pos, goal: round(goal - pos + 360 * (goal < pos), 1)
		#Gets current distance and initially sets previous distance = distance
		distance = chkdist(current_yaw, goal)
		prev_dist = distance
		"""Continues to move while absolute distance is not within angular_error and counterclockwise
		distance is not increasing. NOTE: absolute distance is the shortest distance in either direction,
		while counterclockwise distance is the distance using only counterclockwise movement.
		The angular_error condition was added because the movements tended to end within the first few 
		cycles due to some float error. With the error condition, the movement can only end when inside
		at least the general area of the goal."""
		while distance <= prev_dist or self.get_abs_dist(current_yaw, goal) > riu.angular_error:
			if estop_flag:
				self.publisher.publish(Mover.stop_msg)
			else:
				#Construct and publish left turn message
				twist_msg = Twist()
				twist_msg.angular.z = riu.turn_rate
				self.publisher.publish(twist_msg)
				#If distance to goal is less than half the initial distance, publish the half done message
				if distance <= half_goal and not halfway_flag:
					halfway_flag = True
					self.status_pub.publish(String("half"))
				#Update current position
				with self.move_state_lock:
					current_yaw = (math.degrees(move_state['yaw']) + 360) % 360
				#Set previous distance, then update distance based on new position
				prev_dist = distance
				distance = chkdist(current_yaw, goal)
			rospy.sleep(.2)
		#After loop exit, publish stop message and send done message to cmd_queue
		self.publisher.publish(Mover.stop_msg)
		self.status_pub.publish(String("done"))
		signal.alarm(Mover.ready_message_interval) #Restart timer

	def right(self, param):
		"""If param == 0, sets turn angle to default value.
		Converts current position angle from radians to degrees.
		Converts negative angles to positive. COntinues to turn left until the 
		current distance to the goal is greater than the previous distance, meaning that the 
		goal has been passed."""
		global estop_flag, move_state
		#If input angle is zero, set angle to default
		if param:
			angle = param
		else:
			angle = riu.default_angle

		signal.alarm(0) #Disable timer interrupt for the duration of the movement
		#safely grab current yaw
		with self.move_state_lock:
			current_yaw = (math.degrees(move_state['yaw']) + 360) % 360
		#Set goal to yaw+angle. Add 360 then mod to account for negative angles but avoid going over 360
		goal = (current_yaw - angle + 360) % 360
		if self.angle_lock:
			if goal >= 315 and goal < 45:
				goal = self.zeroed_angle
			elif goal >= 45 and goal < 135:
				goal = self.zeroed_angle + 90
			elif goal >= 135 and goal < 225:
				goal = self.zeroed_angle + 180
			elif goal >= 225 and goal < 315:
				goal = self.zeroed_angle + 270
		goal = goal % 360
		half_goal = (current_yaw - angle/2 + 360) % 360
		halfway_flag = False #used to flag if we've already sent out a halfway message
		#Anonymous function that calculates the current clockwise distance to the goal
		chkdist = lambda pos, goal: round(pos - goal + 360 * (goal > pos), 1)
		#Gets current distance and initially sets previous distance = distance
		distance = chkdist(current_yaw, goal)
		prev_dist = distance
		"""Continues to move while absolute distance is not within angular_error and clockwise
		distance is not increasing. NOTE: absolute distance is the shortest distance in either direction,
		while clockwise distance is the distance using only clockwise movement.
		The angular_error condition was added because the movements tended to end within the first few 
		cycles due to some float error. With the error condition, the movement can only end when inside
		at least the general area of the goal."""
		while distance <= prev_dist or self.get_abs_dist(current_yaw, goal) > riu.angular_error:
			if estop_flag:
				self.publisher.publish(Mover.stop_msg)
			else:
				#Build and publish right turn message
				twist_msg = Twist()
				twist_msg.angular.z = -1 * riu.turn_rate
				self.publisher.publish(twist_msg)
				#If distance to goal is less than half the initial distance, publish the half done message
				if distance <= half_goal and not halfway_flag:
					halfway_flag = True
					self.status_pub.publish(String("half"))
				#Update current position
				with self.move_state_lock:
					current_yaw = (math.degrees(move_state['yaw']) + 360) % 360
				#Update previous distance, then update distance based on current position
				prev_dist = distance
				distance = chkdist(current_yaw, goal)
			rospy.sleep(.2)
		#After loop end, send stop message and send done message to cmd_queue	
		self.publisher.publish(Mover.stop_msg)
		self.status_pub.publish(String("done"))
		signal.alarm(Mover.ready_message_interval) #Restart timer

	def get_abs_dist(self, pos1, pos2):
		"""Gets shortest angular distance between two positions regardless of direction"""
		return min(abs(pos1 - pos2), abs(pos1 - pos2 + 360))

	def scan(self, param):
		"""Calls left 355 to approximate turning in a complete circle"""
		self.left(355)

	def forward(self, param):
		"""Calls linear_move. If no parameter, defaults to default_dist"""
		if param:
			self.linear_move(param * .3048)
		else:
			self.linear_move(riu.default_dist * .3048)


	def backward(self, param):
		"""Calls linear_move. Changes the input parameter to a negative value. If no parameter, defaults to default_dist"""
		if param:
			self.linear_move(-1 * param * .3048)
		else:
			self.linear_move(-1 * riu.default_dist * .3048)
	
	def track(self, param):
		if param:
			self.linear_track(param * .3048)
		else:
			self.linear_track(riu.default_dist * .3048)

	def rtrack(self, param):
		if param:
			self.linear_track(-1 * param * .3048)
		else:
			self.linear_track(-1 * riu.default_dist * .3048)

	def linear_track(self, dist):
		"""
			Checks the tracking variable updated by the tracker callback. If no correction is needed, sends a linear twist message.
			If correction is needed, sends a left or right angular twist as appropriate. Acquires a lock on the move state to update its position.
			Checks for estop every cycle. Disables ready messages for duration of movement. Stops moving after absolute distance from start is equal to 
			the given distance.
		"""
		global estop_flag, move_state

		#Disable timer interrupt, reset halfway flag, set target distance
		signal.alarm(0) 
		halfway_flag = False

		#Set starting position
		with self.move_state_lock:
			start_x, start_y, start_z = move_state['x'], move_state['y'], move_state['z']
		#Set current position initially to start position
		current_x, current_y, current_z = start_x, start_y, start_z
		#Check if the distance travelled is greater than the goal distance
		while math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2 + (current_z - start_z)**2) < abs(dist):
			#Check if the estop flag is set, if so, kill movement
			if estop_flag:
				self.publisher.publish(Mover.stop_msg)
			else:
				twist_msg = Twist()
				if dist < 0:
					if self.correction == riu.no_correction:
						twist_msg.linear.x = -1 * riu.move_rate
					else:
						twist_msg.linear.x = -1 * riu.move_rate/2
					if self.correction == "left":
						twist_msg.angular.z = -1 * riu.turn_rate/2
					elif self.correction == "right":
						twist_msg.angular.z = riu.turn_rate/2
				#If distance goal is positive, move forward
				elif dist > 0:
					if self.correction == riu.no_correction:
						twist_msg.linear.x = riu.move_rate
					else:
						twist_msg.linear.x = riu.move_rate/2
					if self.correction == "left":
						twist_msg.angular.z = riu.turn_rate/2
					elif self.correction == "right":
						twist_msg.angular.z = -1 * riu.turn_rate/2

				self.publisher.publish(twist_msg)
				#Check if the current movement is half completed, if so, send a Half message and set flag to avoid message duplication
				if (math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2 + (current_z - start_z)**2) >= abs(dist)/2
					and not halfway_flag):
					halfway_flag = True
					self.status_pub.publish(String("half"))

				#update current_x, current_y, and current_z (using local variables to be thread safe)
				with self.move_state_lock:
					current_x = move_state['x']
					current_y = move_state['y']
					current_z = move_state['z']
			rospy.sleep(.2)
		self.publisher.publish(Mover.stop_msg)
		self.status_pub.publish(String("done"))
		signal.alarm(Mover.ready_message_interval)

	def linear_move(self, dist):
		"""Moves the robot a distance equal to dist. Checks for estop on each iteration. Publishes a Done message after completion and 
			a Half message when the current distance is equal to half of the goal distance."""
		global estop_flag, move_state
		signal.alarm(0) #Disable timer interrupt for the duration of the movement
		halfway_flag = False
		
		with self.move_state_lock:
			start_x, start_y, start_z = move_state['x'], move_state['y'], move_state['z']
		current_x = start_x
		current_y = start_y
		current_z = start_z
		#While the distance travelled is less than target distance
		while math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2 + (current_z - start_z)**2) < abs(dist):
			#Check if the emergency stop flag is set, if so, break the current loop and reset velocity	
			if estop_flag:
				self.publisher.publish(Mover.stop_msg)
			else:
				#If the distance goal is negative, move backward
				if dist < 0:
					#Send negative velocity
					twist_msg = Twist()
					twist_msg.linear.x = -1 * riu.move_rate
					self.publisher.publish(twist_msg)
				#If distance goal is positive, move forward
				elif dist > 0:
					#Send positive velocity
					twist_msg = Twist()
					twist_msg.linear.x = riu.move_rate
					self.publisher.publish(twist_msg)
				#Check if the current movement is half completed, if so, send a Half message and set flag to avoid message duplication
				if (math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2 + (current_z - start_z)**2) >= abs(dist)/2
					and not halfway_flag):
					halfway_flag = True
					self.status_pub.publish(String("half"))
				#update current_x, current_y, and current_z (using local variables to be thread safe)
				with self.move_state_lock:
					current_x = move_state['x']
					current_y = move_state['y']
					current_z = move_state['z']
			rospy.sleep(.2)
				
		#previously had while, finally block -> illegal syntax in python.  Just moved to outside loop.
		self.publisher.publish(Mover.stop_msg)
		self.status_pub.publish(String("done"))
		signal.alarm(Mover.ready_message_interval)

if __name__ == '__main__':
	try:
		mover = Mover()
	except Exception as er:
		print("Mover module failed to initialize: " + str(er))
	else:
		mover.run()
