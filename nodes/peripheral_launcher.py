#!/usr/bin/env python

import roslib; roslib.load_manifest("turtlebot_code")
import rospy

import os

__authors__ = ["Alex Lalejini"]
__maintainer__ = "Alex Lalejini"
__date__ = "3/6/14"

class Peripheral_Launcher(object):

	#add terminal bringup commands for whatever peripheral software you want to run in 'to_bo_launched'
	to_be_launched = ["vlc"]

	def __init__(self):
		rospy.init_node('peripheral_launcher')

		self.launch_peripherals()

	def launch_peripherals(self):
		'''
		Loops through to_be_launched and tries to run each system call/launch each application
		'''
		for command in Peripheral_Launcher.to_be_launched:
			try:
				os.system(str(command) + " &")
			except Exception as e:
				print("Failed to launch: " + str(command))
				print(" -- Error: " + str(e))

if __name__ == "__main__":
	Peripheral_Launcher()

	

