#!/usr/bin/python

"""serial_server.py: Sends commands to the LED array via serial"""

import rospy
import roslib
roslib.load_manifest("turtlebot_code")
import robot_intent_utilities as riu
from turtlebot_code.msg import Move_Cmd
from std_msgs.msg import String

import serial
import atexit
import sys
import signal
import time

__authors__ = ["Dexter Duckworth", "Alex Lalejini"]
__maintainer__ = "Dexter Duckworth"
__date__ = "10/7/13"

"""
Note: No writeTimeout is specified for led_serial (using default None value).
		- There should be no chance of SerialTimeoutException from write calls (pySerial API).
"""

class serial_server(object):

	#Dictionary of relevant commands and the corresponding LED pending serial command
	pending_cmds = {"scan": "o", "left": "l", "right": "r", "forward": "f", "backward": "e", "stop": "t", "estop":"T", "track":"f", "rtrack":"e"}
	#Same as above, but for LED active serial commands 
	active_cmds = {"scan": "O", "left": "L", "right": "R", "forward": "F", "backward": "E", "stop": "T", "estop": "T", "track":"F", "rtrack":"E"}
	
	handshake_cmd = '@'
	
	serial_retry_time = 5 #Time between serial port searches (in seconds)
	handshake_response_time = rospy.Duration(0, 100000000) #Time to give device to respond to a handshake (in nanoseconds)
	proper_handshake = 'LED-01\r\n' 
	led_serial = None #used to reference LED's serial connection
	
	def __init__(self):
		rospy.init_node('serial_server')
		rospy.Subscriber('/robot_intent/commands', Move_Cmd, self.callback_cmd)
		atexit.register(self.cleanup)
		self.last_command = 'O' #will store most recent command (ignoring estops) for the purpose of displaying correct command after a resume

		while not self.initialize_serial() and not rospy.is_shutdown():	
			#Could not find LED array.  Log exception and retry.
			riu.log_exception("Failed to find LED array serial port. Retrying in " \
			+ str(serial_server.serial_retry_time) + " second(s).", "serial_server: init")
			rospy.sleep(serial_server.serial_retry_time)
		#On init, send a random character to blank out the LED display
		if not rospy.is_shutdown():
			self.wipe_screen()
		
	"""
		This function handles Move_Cmd messages from the /commands topic.
	"""
	def callback_cmd(self, data):
		cmd = self.retrieve_cmd(data.pending, data.action)
		if (not cmd == 'null'):
			self.serial_print(cmd)
			#store last command unless it's an estop (want to remember last command before an estop)
			if (data.action != "estop"):
				self.last_command = cmd

	"""
		Attempt to handshake with the LED array on the first 10 serial ports.
		If successful handshake, initialize led_serial and return true.
		Upon failure, return false.
	"""
	def initialize_serial(self):
		for i in range(0, 10):
			port = '/dev/ttyACM'+str(i)
			try:
				serial_server.led_serial = serial.Serial(port)
				time.sleep(2) #Give arduino time to reset!
			except:
				continue #Failed to open this port.  Move on.
			else:
				if (self.handshake()):
					#Found LED array!
					return True
				else:
					#Wrong port!
					riu.log_exception("Handshake failed", "serial_server: initialize_serial")
					serial_server.led_serial.close()
		#Failed to find LED array.
		return False
	
	"""
		Attempt to handshake with port.  Return true if successful, false otherwise.
	"""	
	def handshake(self):
		self.serial_print(serial_server.handshake_cmd)
		#Give device time to respond
		rospy.sleep(serial_server.handshake_response_time)
		#Read serial port's response
		response = serial_server.led_serial.read(serial_server.led_serial.inWaiting())
		return (response == serial_server.proper_handshake)
		
	"""
		Retrieves action's corresponding serial command.  If not an action we care about, return 'null'
	"""	
	def retrieve_cmd(self, pending, action):
		cmd = 'null'
		try:
			action = action.split(' ')[0] #we just want the command, not it's arguments
			if action == 'resume':
				#if we see a resume, display last known command
				cmd = self.last_command
			elif pending:
				cmd = serial_server.pending_cmds[action]
			else:
				cmd = serial_server.active_cmds[action]
		except KeyError as e:
			pass
		return cmd
	
	"""
		This function is called on program exit.
	"""	
	def cleanup(self):
		try:
			#try to close our serial connection
			serial_server.led_serial.close()
		except:
			#led_serial must be pointing to None (no serial connection to close), which is fine.
			pass
			
	def serial_print(self, data):
		if serial_server.led_serial.isOpen():
			#Send serial data with ascii encoding
			serial_server.led_serial.write(data.encode('ascii'))
		else:
			riu.log_exception("Cannot find serial port", "serial_server: serial_print")

	"""Sends a random character in order to wipe the LED display"""
	def wipe_screen(self):
		if serial_server.led_serial.isOpen():
			#Send serial data with ascii encoding
			serial_server.led_serial.write('C'.encode('ascii'))
		else:
			riu.log_exception("Cannot find serial port", "serial_server: wipe_screen")
		
if __name__ == '__main__':
	try:
		serial_server()
		rospy.spin()
	except rospy.ROSInterruptException as er:
		riu.log_exception(er, "serial_server")

