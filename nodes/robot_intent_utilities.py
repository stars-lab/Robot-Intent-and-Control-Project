#!/usr/bin/env python

"""robot_intent_utilities.py: contains commonly used utility functions for the robot intent project."""

__authors__ = ["Dexter Duckworth"]
__maintainer__ = "Dexter Duckworth"
__date__ = "10/7/13"

import roslib
import rospy
from std_msgs.msg import Header

"""FUNCTIONS"""

"""Prints an error message to the ROS log along with the module
	where the error originated."""
def log_exception(error, name = ""):
	rospy.loginfo("From {}: ".format(name)  + str(error))
	
def build_header(seq = 0, frame = ""):
	hdr = Header()
	hdr.stamp = rospy.Time.now()
	hdr.seq = seq
	hdr.frame_id = frame
	return hdr

"""CONSTANTS"""

"""Stores a list of all valid commands for the turtlebot"""
valid_cmds = ["left", "right", "forward", "backward", "scan", "estop", "track", "resume", "pause", "unpause", "stop", "clear", "rtrack"]
interrupts = ["estop", "resume", "reset", "pause", "unpause", "clear"]

"""
	Stores the standard no correction message sent from tracker if it calculates no necessary correction
	This command should be unique from all cmds in valid_cmds
"""
no_correction = "none"


#Mover Constants
turn_rate = .5 #Positive turns left, negative turns right
move_rate = .2 #Positive moves forward, negative moves back
angular_error = 10 #How close the robot's final position must be to its goal, in degrees
default_angle = 90 #Degrees
default_dist = 1 #Meters
ready_message_interval = 2
