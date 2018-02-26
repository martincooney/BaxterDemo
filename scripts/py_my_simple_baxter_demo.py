#!/usr/bin/env python

#Copyright 2018 Martin Cooney 
#This file is subject to the terms and conditions 
#defined in file 'Readme.md', which is part of this 
#source code package. 

import rospy
from std_msgs.msg import String
import time
import os, sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import cv2
import cv_bridge
from sensor_msgs.msg import (
    Image,
)
import baxter_interface
from baxter_interface import CHECK_VERSION
import subprocess

class demo_baxter_interface:
	def __init__(self):

		#flags
		self.speechRecognitionFlag = False
		self.imageFlag = False

		#subscribers and publishers
		self.sub = rospy.Subscriber('to_baxter', String, self.buttonClickedCallback)
		self.subSpeech = rospy.Subscriber('/recognizer/output', String, self.speechRecognitionCallback)
		image_topic = "/cameras/left_hand_camera/image"
		self.subCam = rospy.Subscriber(image_topic, Image, self.image_callback)	
		self.bridge = cv_bridge.CvBridge()

		#timing
		self.r=rospy.Rate(10)
		self.r.sleep()

		#sounds
		self.soundhandle = SoundClient()
		self.yawnSoundPath = "/home/turtlebot/sounds/yawn.wav" 				#Please change to where the files are on your computer

		#for the robot's display
		self.face_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
		img = cv2.imread("/home/turtlebot/robot_faces/baxter_neutral.png")		#Please change to where the files are on your computer
		self.neutralFace = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
		img = cv2.imread("/home/turtlebot/robot_faces/baxter_happy.png")		#Please change to where the files are on your computer
		self.happyFace = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
		img = cv2.imread("/home/turtlebot/robot_faces/blink.png")			#Please change to where the files are on your computer
		self.blinkFace = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
		img = cv2.imread("/home/turtlebot/robot_faces/switch.png")			#Please change to where the files are on your computer
		self.switchFace = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
		img = cv2.imread("/home/turtlebot/robot_faces/capacitor.png")			#Please change to where the files are on your computer
		self.capacitorFace = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")	
		self.face_pub.publish(self.blinkFace) #robot starts asleep (so human can talk before demoing)
 
		#define arm poses
		self.rs = baxter_interface.RobotEnable(CHECK_VERSION) #for enabling
		self.rightArmNeutralPose = {'right_s0': 0.8053399136398423, 'right_s1': -0.1622184683188825, 'right_w0': -0.13077186216723152, 'right_w1': -0.22012624306155687, 'right_w2': 0.02032524543948173, 'right_e0': -0.017640779060682257, 'right_e1': 1.7437526606287441}
		self.leftArmNeutralPose = {'left_w0': -0.33555829734993425, 'left_w1': -0.1580000211521976, 'left_w2': 0.06634466907604414, 'left_e0': -0.051004861197190006, 'left_e1': 1.5957235145978017, 'left_s0': -0.9503010980950138, 'left_s1': -0.060975736318445196}
		self.leftArmGreetingPose = {'left_w0': -2.1736507764336315, 'left_w1': -0.26422819071326253, 'left_w2': 0.09664078963678106, 'left_e0': 0.21935925266761416, 'left_e1': -0.051004861197190006, 'left_s0': -0.2956747968649135, 'left_s1': -0.8782040010643993}
		self.right_limb = baxter_interface.Limb('right')
		self.left_limb = baxter_interface.Limb('left')

	def image_callback(self, msg):
    		#print("Received an image!") #uncomment for debugging
		if(self.imageFlag == True):
    			try:
            			cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        		except cv_bridge.CvBridgeError, e:
            			print(e)
        		else:
            			#get image, process as you wish, then send to robot's display
				resized_frame = cv2.resize(cv2_img, (1024, 600))
				currentFace = cv_bridge.CvBridge().cv2_to_imgmsg(resized_frame, encoding="bgr8")
				self.face_pub.publish(currentFace)

		self.r.sleep()

	#speech recognition callback
	def speechRecognitionCallback(self, data):
		#print data.data #uncomment for debugging
		if(self.speechRecognitionFlag == False):
			print "heard something but ignoring"
		else:
			#do something based on recognized keywords
			#here we recognize the names of some common electronics components and show them in the robot's display
			if(data.data=='switch'):
				print "heard switch!"
				self.face_pub.publish(self.switchFace)
			elif(data.data=='capacitor'):
				print "heard capacitor!"
				self.face_pub.publish(self.capacitorFace)


	def buttonClickedCallback(self, data):

		#print data.data #uncomment for debugging

		if(data.data=='00'):
        		if not self.rs.state().enabled:
            			print("Enabling robot...")
            			self.rs.enable()

			self.left_limb.move_to_joint_positions(self.leftArmNeutralPose)
    			self.right_limb.move_to_joint_positions(self.rightArmNeutralPose)

			self.face_pub.publish(self.happyFace)
			self.soundhandle.say('Hello!')

			#look around
			baxter_interface.Head().set_pan(-0.5)
			time.sleep(0.5)
			baxter_interface.Head().set_pan(0.5)
			time.sleep(0.5)
			baxter_interface.Head().set_pan(0.0)
			time.sleep(2)

			self.soundhandle.say('Welcome!')
			self.left_limb.move_to_joint_positions(self.leftArmGreetingPose)
			time.sleep(2)
			self.face_pub.publish(self.blinkFace) #blink
			time.sleep(0.2)
			self.face_pub.publish(self.happyFace)

			self.soundhandle.say('My name is Baxter. I am a social robot in training')
			time.sleep(5)
			self.face_pub.publish(self.blinkFace) #blink
			time.sleep(0.2)
			self.face_pub.publish(self.happyFace)

			self.soundhandle.say('and I am trying to help out in education and research.')
			time.sleep(5)

			self.soundhandle.say('To interact, I have two 7 degree of freedom arms, an omnidirectional base, speakers, and a display.')
			time.sleep(10)
			self.face_pub.publish(self.blinkFace) #blink
			time.sleep(0.2)
			self.face_pub.publish(self.happyFace)

			self.soundhandle.say('To sense, I also have 13 sonar sensors around my head, infrared range sensors and cameras in my hands, force sensors in my arms, a laser and IMU in my base, and a microphone.')
			self.left_limb.move_to_joint_positions(self.leftArmNeutralPose)
    			self.right_limb.move_to_joint_positions(self.rightArmNeutralPose)
			time.sleep(12)
			self.face_pub.publish(self.blinkFace) #blink
			time.sleep(0.2)
			self.face_pub.publish(self.happyFace)

			self.soundhandle.say('Hope you are having a nice visit!')
			time.sleep(4)

		elif(data.data=='01'):
        		if not self.rs.state().enabled: 
				print("Enabling robot...")
            			self.rs.enable()

			self.left_limb.move_to_joint_positions(self.leftArmNeutralPose)
    			self.right_limb.move_to_joint_positions(self.rightArmNeutralPose)
			time.sleep(3)
			self.face_pub.publish(self.neutralFace)
			self.soundhandle.say('Recording a motion')
			time.sleep(1)
			#self.soundhandle.playWave(self.yawnSoundPath); #uncomment to play some sound file
			subprocess.call(['rosrun baxter_examples joint_recorder.py -f test_motion'], shell=True)		
			time.sleep(5)

		elif(data.data=='02'):
        		if not self.rs.state().enabled: 
				print("Enabling robot...")
            			self.rs.enable()
			self.left_limb.move_to_joint_positions(self.leftArmNeutralPose)
    			self.right_limb.move_to_joint_positions(self.rightArmNeutralPose)
			time.sleep(3)
			self.face_pub.publish(self.neutralFace)
			self.soundhandle.say('Playing back a motion')
			time.sleep(1)
			#self.soundhandle.playWave(self.yawnSoundPath); #uncomment to play some sound file	
			subprocess.call(['rosrun baxter_examples joint_position_file_playback.py -f test_motion'], shell=True) 
			time.sleep(5)

		elif(data.data=='03'):
			print 'Toggle image processing'
        		if (self.imageFlag == True):
        			self.imageFlag = False
				self.face_pub.publish(self.neutralFace)
				self.soundhandle.say("Stopped showing camera feed") 
        		else:
				self.soundhandle.say("Showing camera feed") 
        			self.imageFlag = True

		elif(data.data=='04'):
			print 'Toggle speech recognition'
        		if (self.speechRecognitionFlag == True):
        			self.speechRecognitionFlag = False
				self.soundhandle.say("Okay, I won't listen anymore!") 
				time.sleep(7)
				self.face_pub.publish(self.neutralFace)

        		else:
        			if self.rs.state().enabled: 
            				print("Disabling robot...")
            				self.rs.disable()
				self.soundhandle.say('I am listening now!') 
				time.sleep(7) 					#wait so the robot won't hear its own words
        			self.speechRecognitionFlag = True

		elif(data.data=='05'):
			self.face_pub.publish(self.neutralFace)
			self.soundhandle.playWave(self.yawnSoundPath);
			time.sleep(3)
			self.soundhandle.say('I will go to sleep now! Good night.') 
			time.sleep(2)
			self.face_pub.publish(self.blinkFace)
			sleep(2)
	
		elif(data.data=='06'):
			#free place to add your own behaviors
			time.sleep(1)

		elif(data.data=='07'):
			subprocess.call(['rosrun baxter_examples joint_velocity_puppet.py -l right'], shell=True) 
			time.sleep(2)

		elif(data.data=='08'):
			self.soundhandle.say('Going back to initial state') 
			self.face_pub.publish(self.blinkFace)
        		if not self.rs.state().enabled: 
				print("Enabling robot...")
            			self.rs.enable()
			self.left_limb.move_to_joint_positions(self.leftArmNeutralPose)
    			self.right_limb.move_to_joint_positions(self.rightArmNeutralPose)
			time.sleep(2)

		elif(data.data=='09'):
			self.soundhandle.say('test') 
			time.sleep(2)

		elif(data.data=='10'): 
 			print 'enable'
			self.rs.enable()

		elif(data.data=='11'): 
 			print 'disable'
			self.rs.disable()

		time.sleep(0.01)

def sleep(t):
	try:
        	rospy.sleep(t)
	except KeyboardInterrupt:
		sys.exit()
	except:
		pass

def listener():
	rospy.init_node('baxterDemo', anonymous=True)
	my_baxter_interface = demo_baxter_interface()


	while not rospy.is_shutdown():
		my_baxter_interface.r.sleep()


if __name__== '__main__':

    	print '-------------------------------------'
    	print '-   Baxter simple controller demo   -'
	print '-   for art and teaching (python)   -'
    	print '-   DEC 2016, HH, Martin            -'
    	print '-------------------------------------'

	try:
		listener()
	except rospy.ROSInterruptException:
		pass
	finally:
		pass

