#!/usr/bin/env python

"""socket_server.py: Sends and receives data from the Android tablet and Kinect"""
import errno
import re
import signal
import socket
import SocketServer as socketserver
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
import atexit


import rospy
import roslib
roslib.load_manifest("turtlebot_code")

import robot_intent_utilities as riu
import os
from turtlebot_code.msg import Move_Cmd, Move_Request
from std_msgs.msg import Header

__authors__ = ["Dexter Duckworth","Paul Barrett"]
__maintainer__ = "Dexter Duckworth"
__date__ = "10/8/13"

def isValidIPV4(IP):
	#check that a string is a valid IP. this might be more pythonic with thrown errors, I'm not sure.
	validIP=re.compile("^(?:[0-9]{1,3}\.){3}[0-9]{1,3}$")
	if(validIP.search(IP)==None):
		return False
	for i in IP.split('.'):
		if(int(i)>255):
			return False
	return True
	
def getIP():
	wlanRE=re.compile('eth0.*',re.DOTALL)
	inetRE=re.compile('inet addr:(?:[0-9]{1,3}\.){3}[0-9]{1,3}')
	validIP=re.compile("(?:[0-9]{1,3}\.){3}[0-9]{1,3}")
	return validIP.search(inetRE.search(wlanRE.search(subprocess.check_output('/sbin/ifconfig')).group(0)).group(0)).group(0)

	

class socket_server(socketserver.ThreadingMixIn,socketserver.TCPServer):
	command_map = {"track":"forward", "rtrack":"backward"}
	def __init__(self):
		'''
		TODO: This is for manual input of local IP. Should probably be in a try catch in getIP as a failsafe or something.
		while(1):
			self.host=raw_input("Enter host IP or blank for localhost:")
			self.host=self.host.strip()
			if(self.host=='' or isValidIPV4(self.host)):
				break
			else:
				print "Invalid IP address."
		'''
		atexit.register(self.ctrlCHandler)
		self.seq=0
		self.host=getIP()
		self.PORT=9000
		self.clientSockets=set()
		#Standard ros stuff: init node, subscribe to things and create publishers
		rospy.init_node('socket_server')
		rospy.Subscriber('/robot_intent/commands', Move_Cmd, self.callback_command)
		self.pub = rospy.Publisher('/robot_intent/requests', Move_Request)
		loopCount=0
		while not rospy.is_shutdown():
			try:
				#Create a server with a message handler
				socketserver.TCPServer.__init__(self,(self.host, self.PORT), AndroidMessageHandler)
				#Start the server in a separate thread
				self.serverThread=threading.Thread(target=self.serve_forever)
				self.serverThread.daemon=True #This makes the thread die when the main thread does.
				self.serverThread.start()
				self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #This should make it where we can reuse the socket no matter what... 
			except socket.error as e:
				if(e.errno==errno.EADDRINUSE): #check for the specific error that comes with the socket needing to timeout
					if(loopCount>180): #eventually timeout so we don't keep trying forever
						print "Could not connect due to address being in use."
						sys.exit(1)
					elif(loopCount==0):
						riu.log_exception(e,"socket_server")
						print "Trying again, please wait. (If you have seen this message for more than 3 minutes, restart the socket server.)"
					loopCount+=1
					time.sleep(1) #However, this still seems iffy, which is why the while loop is here. the sleep is here so as to not flood the log.
				else: #if it's not that error it's something else, inform the user and exit
					riu.log_exception(e,"socket_server")
					sys.exit(1)
			except:
				riu.log_exception(sys.exc_info()[0],"socket_server")
				time.sleep(1)
			else:
				print "Local socket opened on:"+str(self.socket.getsockname())
				break #We finally got to bind our socket.
	def callback_command(self,data):
		#React to messages on the commands topic by sending them to the handset
		if(data.action=="null"):
			return #don't do anything with null messages
		
		else:
			command = data.action.split()[0] #Split the command to remove any parameters
			if command in socket_server.command_map.keys(): #Replaces the command with its display equivalent, if necessary
				command = socket_server.command_map[command]
			data.action = command 
		msg = ET.tostring(self.buildCmdMessage(data))+b'\n'
		print "Sending:"+msg,
		for s in self.clientSockets:
			try:
				s.sendall(msg)
			except socket.error as e:
				riu.log_exception(e,"socket_server")
			except AttributeError as e:
				#We see an attribute error if the client socket is None.
				#Might not be a problem with the iterator.
				if(not s):
					riu.log_exception("No device connected.", "socket_server")
				else:
					riu.log_exception(e, "socket_server")
	def buildCmdMessage(self,data):
		#Use a treebuilder to create the xml message.
		builder=ET.TreeBuilder()
		builder.start("xml",{})
		builder.start("timestamp",{})
		builder.data(str(data.header.stamp))
		builder.end("timestamp")
		builder.start("action",{})
		builder.data(data.action)
		builder.end("action")
		builder.start("pending",{})
		builder.data(str(data.pending) )
		builder.end("pending")
		builder.start("source",{})
		builder.data(str(data.source) )
		builder.end("source")
		builder.end("xml")
		return builder.close()
	def ctrlCHandler(self):
		self.socket.shutdown(socket.SHUT_RDWR)
		for sock in self.clientSockets:
			sock.shutdown(socket.SHUT_RDWR)
		sys.exit(1)


class AndroidMessageHandler(socketserver.BaseRequestHandler):
	def handle(self):
		#When a client connects, add it to the set of connected sockets.
		self.server.clientSockets.add(self.request)
		print "Device connected on:"+str(self.request.getpeername())
		print "Connected sockets:",
		for s in self.server.clientSockets:
			print str(s.getpeername()),
		print '\n',
		
		#When a message is received on the socket, handle it by publishing it on the requests topic.
		while not rospy.is_shutdown():
			try:
				line=self.request.recv(4096) #arbitrary buffer size
			except:
				line='' #TODO: handle this properly	
			else:
				lineString=line.decode() #decode from byte string
				if(lineString): #only publish if we are actually receiving data on the socket
					print "Received:"+lineString,
					try:
						xmlMessage=ET.fromstring(lineString)
					except ET.ParseError as e:
						xmlMessage=None
						riu.log_exception(e,"socket_server")
					else:
						if(xmlMessage.find("action").text=="disconnect"):
							break
						else:
							if (xmlMessage.find("action").text.find("track") == -1):
								self.server.pub.publish(Move_Request(riu.build_header(self.server.seq), xmlMessage.find("action").text, "socket_server: "+xmlMessage.find("source").text, 1))
								self.server.seq += 1
							else:
								self.server.pub.publish(Move_Request(riu.build_header(self.server.seq), xmlMessage.find("action").text, "socket_server: "+xmlMessage.find("source").text, 2))
								self.server.seq += 1
	def finish(self):
		self.server.clientSockets.remove(self.request)
		
		try:
			print "Device disconnected from:"+str(self.request.getpeername())
		except:
			print "Device disconnected"
		return socketserver.BaseRequestHandler.finish(self)
	

if __name__ == "__main__":
	try:
		socket_server()
		rospy.spin()
	except rospy.ROSInterruptException as er:
		riu.log_exception(er, "socket_server")
