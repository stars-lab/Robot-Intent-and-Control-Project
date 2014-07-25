#!/usr/bin/python

"""serial_server.py: Sends commands to the LED array via serial"""

import time


import serial



__authors__ = ["Dexter Duckworth", "Alex Lalejini"]
__maintainer__ = "Dexter Duckworth"
__date__ = "10/7/13"

"""
Note: No writeTimeout is specified for led_serial (using default None value).
		- There should be no chance of SerialTimeoutException from write calls (pySerial API).
"""

class serial_server():
	
	def __init__(self):
		
		s = serial.Serial('/dev/ttyACM0', 9600)
		time.sleep(3)
		s.write('T')
		#rospy.sleep(1)
		time.sleep(1)
		print("inWaiting: " + str(s.inWaiting()))
		print("Response: " + str(s.read(s.inWaiting())))
		s.close()

	"""
		This function handles Move_Cmd messages from the /commands topic.
	"""
#	def callback_cmd(self, data):
#		print("This should never actually be printed")


if __name__ == '__main__':
	serial_server()
	#rospy.spin()

