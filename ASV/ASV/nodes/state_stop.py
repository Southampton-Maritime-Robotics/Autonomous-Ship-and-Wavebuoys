#!/usr/bin/env python
##############################################################################
#state_stop.py
#
#Initial code is a simplified subset of state_stop.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#
#Modifications to code
# 21/11/2012	Modified code for ASV	ABP
#
#
#
##############################################################################
#Notes
#
#This code contains the code for the stop state which terminates the ASVs run
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String

#################smach state Stop (defined in mission_script.py)################
class Stop(smach.State):
	def __init__(self, lib):
		smach.State.__init__(self, outcomes=['succeeded'])
                self.__controller = lib
                   		
	def execute(self,userdata):
                global pub
                #Set Up Publisher for Mission Control Log
                pub = rospy.Publisher('MissionStrings', String)               
                str= 'ASV STOP state started at time = %s' %(time.time())
                pub.publish(str)


                #Initiate the stop command in Library high level                
                self.__controller.stop()
                return 'succeeded'
