#!/usr/bin/python

"""odom_server.py: Converts incoming odometry data into Euler coordinates and a Twist"""

# Start up ROS pieces.mv
import roslib; roslib.load_manifest('turtlebot_code')
import rospy
import tf

# ROS messages.
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from turtlebot_code.msg import Converted_Odom
import robot_intent_utilities as riu


__authors__ = ["Dexter Duckworth", "John Kelly", "Alex Lalejini"]
__maintainer__ = "Dexter Duckworth"
__date__ = "10/17/13"


class Odom_Server():
	
	def __init__(self):
		rospy.init_node('odom_server')
		#TODO: probably not okay that Imu type messages and Odometry are handled on the same callback in the same way
		rospy.Subscriber("/odom", Odometry, self.odom_callback)
		self.pub_converted_odom = rospy.Publisher("/robot_intent/odometry", Converted_Odom)
		self.seq = 0

	""" Function takes in the Odometry message and converts it """
	def odom_callback(self, msg):
		#TODO: What exactly does mover.py need sent to it?
		#	- entire 'twist' msg object?
		
		#grab twist message object from message
		twist = msg.twist.twist
		#grab orientation (in quats) object from message
		orientation = msg.pose.pose.orientation
		#Gets the xyz coordinates of the robot
		position = msg.pose.pose.position
		# gets the angular orientation in Eulers
		try:
			#error handling for mystery library function
			euler = self.quat_to_euler(orientation)
		except:
			riu.log_exception("Failed to convert quats to eulers", "odom_server")
		else:
			roll = euler[0] #grab roll
			pitch = euler[1] #grab pitch
			yaw = euler[2] #grab yaw
			# publishes the message
			self.pub_converted_odom.publish(Converted_Odom(riu.build_header(self.seq), roll, pitch, yaw, twist, position))
			self.seq += 1

	""" 
		Converts orientation values from quaternions to Eulers 
		Returns a Euler, which is a 3 tuple (roll, pitch, yaw)
	"""
	def quat_to_euler(self, quat_orientation):
		# quat orientation values from Odometer
		quat_x = quat_orientation.x
		quat_y = quat_orientation.y
		quat_z = quat_orientation.z
		quat_w = quat_orientation.w
		#Transformation from quat values to Eulers
		return tf.transformations.euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])

# Main function.    
if __name__ == '__main__':
    # Initializes the node and name it.
    try:
    	odom_server = Odom_Server()
    	rospy.spin()
    except rospy.ROSInterruptException as er:
    	riu.log_exception(er, "odom_server")

