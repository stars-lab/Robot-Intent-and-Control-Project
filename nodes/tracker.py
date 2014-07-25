#!/usr/bin/env python

"""marker_tracking.py: Tracks visible marker using video feed and sends commands to turtlebot to align turtlebot with the marker """

import roslib; roslib.load_manifest("turtlebot_code")
import rospy
from turtlebot_code.msg import Move_Request
from std_msgs.msg import String
import robot_intent_utilities as riu

import numpy as np
import cv2
import time
import cv2.cv as cv

import zbar
import Image
import signal

#from matplotlib import pyplot as plt

__authors__ = ["Alex Lalejini, John Kelly"]
__maintainer__ = "Alex Lalejini, John Kelly"
__date__ = "11/20/13"

#TODO: REFACTOR CODE ONCE WE KNOW IT WORKS AS DESIRED!
#TODO: ADD ERROR CHECKING/HANDLING
class Track(object):

    def __init__(self):
        rospy.init_node('track')
        self.publisher = rospy.Publisher('/robot_intent/requests', Move_Request)
        self.mover_publisher = rospy.Publisher('/robot_intent/tracking', String)
        self.seq = 0
        #cv2 video capture object
        self.capture = None
        #Marker's acceptable color range (used when thresholding to generate a mask)
        self.bound = [(100, 255), (0, 75), (100, 255)]
        #out of view threshold value - percentage of pixels needed to be set in the mask to indicate a marker is in view
        self.in_view_threshold = 0.0025
        #Number of pixels off center the marker is allowed to be without a correction being necessary
        self.alignment_error = 50

        self.scanner = None
        #QR codes that have been previously seen
        self.last_code = None
    
    #TODO: MAKE SURE WE GRAB THE CORRECT CAMERA, AND MAKE SURE WE ACTUALLY GRAB A CAMERA BEFORE LEAVING THIS FUNCTION
    def initialize_camera(self):
        self.capture = cv2.VideoCapture(1)
        rospy.sleep(2)

    def destroy_camera(self):
        self.capture.release()
    
    def initialize_scanner(self):
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')

    def scan_image(self, raw, width, height):
        # wrap image data
        image = zbar.Image(width, height, 'Y800', raw)

        # scan the image for barcodes
        self.scanner.scan(image)

        # extract results
        for symbol in image:
            # do something useful with results
            #print ('decoded', symbol.type, 'symbol', '"%s"' % symbol.data)
            if(symbol.data == self.last_code):
                pass
                #print("Sybmol has already been seen")
                ## publish command request
            else:
                #print(symbol.data +" has been added to the seen list")
                self.last_code = symbol.data
                self.publish_code(symbol.data)
                #rospy.sleep(5)
        # clean up
        del(image)

    def publish_code(self, code):
        # QR codes should be in the form ID#message
        # Gets rid of the ID at the front of the string
        identifier = code.split("#")[0]
        messages = (code.split("#"))[1:]
        for each in messages:
            print("Publishing: " + str(each.lower()))
            #NOTE: Important that this message's source remains QRScanner! (must be different from tracking source -- 'tracker')
            #QR code ID is important!  Must be included in the source attribute of the msg of the format QRScanner#ID
            self.publisher.publish(Move_Request(riu.build_header(self.seq), each.lower(), "QRScanner#" + str(identifier), 2))
            self.seq += 1
            rospy.sleep(0.25)

    #TODO: Make this code more readable by splitting it into functions!
    def track(self):
        #initialize the capture device
        self.initialize_camera()
        #Initializes the zbar scanner
        self.initialize_scanner()
        current_correction = riu.no_correction
        previous_correction = ""
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = self.capture.read()
            #cv2.imshow('frame2', frame)
            image = Image.fromarray(frame).convert("L")
            width, height = image.size
            raw = image.tostring()
            #image.show()
            #print(raw)
            try:
                self.scan_image(raw, width, height)
            except:
                pass
        
            #Center of camera (Y, X)
            #NOTE: FRAME KEEPS TRACK OF IMAGE IN FORM (Y, X)
            camera_center = (frame.shape[0]/2.0, frame.shape[1]/2.0)
            #frame is stored as a numpy array
        
            #Create a mask for each channel of color (RGB)
            red_range = np.logical_and(self.bound[0][0] < frame[:,:,0], frame[:,:,0] < self.bound[0][1])
            green_range = np.logical_and(self.bound[1][0] < frame[:,:,1], frame[:,:,1] < self.bound[1][1])
            blue_range = np.logical_and(self.bound[2][0] < frame[:,:,2], frame[:,:,2] < self.bound[2][1])
            #Put the three channel masks back together and form
            valid_range = np.logical_and(red_range, green_range, blue_range)
            #Create the final mask
            frame[valid_range] = 255
            frame[np.logical_not(valid_range)] = 0
            
            #grab the number of nonzero pixels in frame (also == to the number of nonzero elements in valid_range)
            nonzero = np.nonzero(valid_range)
            out_of_view_thresh = self.in_view_threshold * frame.shape[0] * frame.shape[1] #if less than in_view_threshold % of the pixels are true, the targ is out of view
            if nonzero[1].size < out_of_view_thresh:
                #Target is out of view! - Send out a no correction message. TODO: Are we sure we want to send out a no correction message?
                current_correction = riu.no_correction
            else:
                #calculate the location of the centroid -- all we care about is the x coordinate
                centroid = sum(nonzero[1])/float(nonzero[1].size)
                dist_error = self.alignment_error #amount of error allowed before right or left turn is suggested
                if camera_center[1] - centroid > dist_error:
                    #print("Move Left")
                    #We need to move left
                    #TODO: refactor such that we do not have to use this super jank way to send correction
                    current_correction =  "left"
                elif camera_center[1] - centroid < -1 * dist_error:
                    #print("Move Right")
                    #Need to move right
                    current_correction = "right"
                else:
                    #Don't need to move left or right, no correction necessary
                    #print("No Correction")
                    current_correction = riu.no_correction

            if current_correction is not previous_correction:
                self.mover_publisher.publish(String(current_correction))
                previous_correction = current_correction
            #Display the resulting frame
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.destroy_camera()   
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tracker = Track()
    tracker.track()

