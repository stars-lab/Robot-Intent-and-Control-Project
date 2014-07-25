#!/usr/bin/env python

"""cmd_queue.py: Manages incoming commands and adds them to a queue to be sent to the mover."""

__authors__ = ["Dexter Duckworth", "Alex Lalejini"]
__maintainer__ = "Dexter Duckworth"
__date__ = "10/7/13"

import roslib; roslib.load_manifest("turtlebot_code")
import rospy
from std_msgs.msg import String

import robot_intent_utilities as riu
from turtlebot_code.msg import Move_Cmd, Move_Request



class cmd_queue(object):
	status_keywords = {"done":"done", "half":"half", "incomplete":"incomplete", "ready":"ready"} #Defines the keywords sent by the mover indicating its status
	min_wait = 4 #Minimum time in seconds that the robot should wait between receiving a command and publishing it
	override_priority = 1 #priority that indicates override command

	def __init__(self):
		rospy.init_node('cmd_queue')
		self.publisher = rospy.Publisher('/robot_intent/commands', Move_Cmd)
		rospy.Subscriber('/robot_intent/requests', Move_Request, self.callback_requests)
		rospy.Subscriber('/robot_intent/mover_status', String, self.callback_status)
		self.pending = []
		self.active = ""
		self.seq = 0
		self.half_flag = False #Indicates whether the queue needs to send a pending message
		self.pause_flag = False #Indicates whether the queue is paused and will not send active messages
		
	
	def callback_requests(self, data):
		"""
		This function handles messages from the /requests topic.
		The function first checks for interrupts before checking if a message comes from a 
		priority source. This is to ensure that interrupts are handled properly.
		"""	
		#Source is currently ignored
		data.action = data.action.lower()
		if self.check_interrupt(data.action):
			self.handle_interrupt(data.action)
		else:
			self.push_pending(data)
			
	def callback_status(self, data):
		"""
		This function handles messages from the /mover_status topic.
		
		NOTE: If the cmd_queue recieves a Done or Ready message while sending a command 
		but before the mover has received the command, the cmd_queue will 
		send another command. 
		"""
		if data.data == cmd_queue.status_keywords["done"] or data.data == cmd_queue.status_keywords["ready"]:
			self.half_flag = False
			#Mover status indicates 'done'. Publish new active command (Front of pending queue) on /commands topic.
			if len(self.pending) > 0 and (rospy.Time.now().secs - self.peek_pending().header.stamp.secs) <= cmd_queue.min_wait:
				#If the next message has been in the queue for less than 5 seconds, do nothing
				pass
			#If the command has been in the queue for the minimum amount of time and the queue is not paused, send active command
			elif not self.pause_flag:
				cmd_src = self.peek_pending_source()
				self.publisher.publish(Move_Cmd(riu.build_header(self.seq), self.pop_pending_command(), False, cmd_src))
				self.seq += 1
		elif data.data == cmd_queue.status_keywords["half"]:
			#Mover status indicates current active command is halfway done.
			#Publish the next command in the pending queue.  Indicate a status of pending.
			if len(self.pending) > 0:
				cmd_src = self.peek_pending_source()
				self.publisher.publish(Move_Cmd(riu.build_header(self.seq), self.peek_pending_command(), True, cmd_src))
				self.seq += 1
			else:
				self.half_flag = True
	
	def pop_pending_command(self):
		"""
		This funcddtion safely pops from the front of the pending action queue.
		If the queue is empty, returns a null string to be handled by the move node
			else returns the popped queue element
		"""
		try:
			self.active = self.pending.pop(0).action
		except IndexError as er:
			riu.log_exception(er, "cmd_queue")
			return "null"
		else:
			return self.active
			
	def peek_pending_command(self):
		"""
		This function safely peeks at the front of the queue and returns what is sees.
		If the queue is empty, returns a null string
		"""
		try:
			return self.pending[0].action
		except IndexError as er:
			riu.log_exception(er, "cmd_queue")
			return "null"

	def peek_pending_source(self):
		"""
		This funcddtion safely peeks at the source of the command at the top of the action queue.
		If the queue is empty, returns a null string.  Else, it returns the peeked source.
		"""
		try:
			return self.pending[0].source
		except IndexError as er:
			riu.log_exception(er, "cmd_queue")
			return "null"
	
	def peek_pending(self):
		"""
		This function safely peeks at the front of the queue and returns what is sees.
		If the queue is empty, returns a null string
		"""
		try:
			return self.pending[0]
		except IndexError as er:
			riu.log_exception(er, "cmd_queue")
			return "null"
	
	def push_pending(self, item):
		"""
		Item is expected to be a message of type Command_Request
		If the new message has a higher priority than the top of the queue, replace top of queue. If the priority is the same, search until you find the last 
		item of equal priority and append the new item after. If the new priority is lower, search until you find the last item of greater or equal priority and append.
		Returns true on success, false on failure
		"""	
		try:
			#If queue is empty, append new item
			if len(self.pending) == 0:
				self.pending.append(item)
				if self.half_flag:
					cmd_src = self.peek_pending_source()
					self.publisher.publish(Move_Cmd(riu.build_header(self.seq), self.peek_pending_command(), True, cmd_src))
					self.seq += 1
					self.half_flag = False
				return True	
			#If new item has override priority, clear the queue and insert at the top.
			elif item.priority == cmd_queue.override_priority:
				self.pending = []
				self.pending.append(item)
				cmd_src = self.peek_pending_source()
				self.publisher.publish(Move_Cmd(riu.build_header(self.seq), self.peek_pending_command(), True, cmd_src))
				self.seq += 1
				return True	
			#If new item has same or lower priority, search until an item with lower priority is found and insert the new item before it
			elif item.priority >= self.peek_pending().priority:
				for i in xrange(1,len(self.pending)):
					if item.priority > self.pending[i]:
						self.pending.insert(i, item)
						return True
				self.pending.append(item)
				return True	
		#Catch and log any exceptions
		except Exception as e:
			riu.log_exception(e, "cmd_queue")
			return False
		
	def check_interrupt(self, item):
		"""
		Returns true if given command, item, is found in cmd_queue's static interrupt dictionary
		or if the item's source is a priority source.
		"""	
		return item in riu.interrupts
	
	def handle_interrupt(self, interrupt):
		"""
		Handles interrupts.
		"""	
		if interrupt == "estop":
			self.publisher.publish(Move_Cmd(riu.build_header(self.seq), "estop", False, "interrupt"))
			self.seq += 1
		elif interrupt == "resume":
			self.publisher.publish(Move_Cmd(riu.build_header(self.seq), "resume", False, "interrupt"))
			self.seq += 1
		elif interrupt == "pause":
			self.pause_flag = True
		elif interrupt == "unpause":
			self.pause_flag = False
		elif interrupt == "clear":
			self.pending = []
			
if __name__ == "__main__":
	try:
		cmd_queue()
		rospy.spin()
	except rospy.ROSInterruptException as er:
		riu.log_exception(er, "cmd_queue")
