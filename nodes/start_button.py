#!/usr/bin/env python

"""start_button.py: Listens for button to be pressed. When pressed the first time, sends a start message.
	When pressed a second time, sends a stop message."""

import roslib; roslib.load_manifest("turtlebot_code")
import rospy
from turtlebot_code.msg import Move_Request
import robot_intent_utilities as riu

__authors__ = ["Dexter Duckworth"]
__maintainer__ = "Dexter Duckworth"
__date__ = "12/4/13"

class StartButton(object):

	active_button = 0;

	def __init__(self):
		rospy.init_node('start_button')
		self.is_running = False;
		self.seq = 0;
		#self.sub = rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.callback)
		self.pub = rospy.Publisher("/robot_intent/requests", Move_Request)

	"""def callback(self, buttons):
		if buttons["state"] == 1 and buttons["button"] == active_button:
			if self.is_running:
				self.pub.publish(Move_Request(riu.build_header(self.seq), "stop", "start_button"))
			else:
				self.pub.publish(Move_Request(riu.build_header(self.seq), "start", "start_button"))
			self.seq += 1
			self.is_running = not self.is_running"""
	def run(self):
		self.pub.publish(Move_Request(riu.build_header(self.seq), "start", "start_button"))
		self.seq += 1
		self.is_running = not self.is_running
	
if __name__ == "__main__":
	try:
		startbtn = StartButton()
		rospy.sleep(5)
		startbtn.run()
	except rospy.ROSInterruptException as er:
		riu.log_exception(er, "start_button")
