#!/usr/bin/env python

"""text_parser.py: Manages lists of commands and sends them to the command_queue. 
					Lines beginning with # are treated as comments."""

import roslib; roslib.load_manifest("turtlebot_code")
import rospy

from turtlebot_code.msg import Move_Request
from collections import deque
import os
import sys
import robot_intent_utilities as riu

__authors__ = ["Dexter Duckworth","Alex Lalejini"]
__maintainer__ = "Dexter Duckworth"
__date__ = "10/5/13"

class Parser(object):
	
	#Folder for movement command files. Currently hard coded.
	text_folder = "/home/turtlebot/catkin_workspace/src/robot-intent-robot/move_files/" 
	#Length  to wait if no arguments are passed with delay command (seconds)
	default_delay = 1
	
	def __init__(self):
		self.action_queue = deque()
		rospy.init_node('text_parser')
		self.publisher = rospy.Publisher('/robot_intent/requests', Move_Request)
		self.seq = 0
	
	"""Checks if each item in the folder is a file, then it reads the file"""
	def get_files(self):
		# Iterates through each item in the directory
		for item in os.listdir(Parser.text_folder):
			item = Parser.text_folder + item
			#If the item is a file and ends in .txt, read the item"
			if os.path.isfile(item) and item.split('.')[-1] == 'txt':
				self.read_file(item)
	
	"""Opens a file and retrieves each line"""
	def read_file(self, file_name):
		with open(file_name) as text:
			for line in text:
				#Omit blank lines and lines beginning with '#'
				if line is not '/n' and line[0] is not '#':
					#Strips newline characters. Treats each line as one command.
					self.action_queue.appendleft(line.replace('\n', '').lower())
				
	"""Checks that the folder is a valid path then gets text files and 
		reads commands to the queue"""
	def parse(self):
		#If path is not valid, log error and exit
		if not os.path.isdir(Parser.text_folder):
			riu.log_exception("Path not found", "text_parser")
			sys.exit(1)
		else:	
			self.get_files()
			#While queue is not empty, parse queue
			while len(self.action_queue):
				cmd = self.action_queue.pop()
				#If command is a delay, wait for the appropriate duration
				if cmd.split(' ')[0] == "delay":
					try:
						delay_len = float(cmd.split(' ')[-1])
					except:
						delay_len = Parser.default_delay
					finally:
						rospy.sleep(delay_len)
				#If not a delay and also a valid command, publish command
				elif cmd.split(' ')[0] in riu.valid_cmds:
					self.publisher.publish(Move_Request(riu.build_header(self.seq), cmd, "text_parser", 2))
					self.seq += 1

if __name__ == "__main__":
	parser = Parser()
	parser.parse()
		
