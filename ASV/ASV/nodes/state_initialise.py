#!/usr/bin/env python
##############################################################################
#state_initialise.py
#
#Initial code is a simplified subset of state_initialise.py from DelphinROSv2
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
#This state should be run at the start of every mission to ensure that all critical systems have come online
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String


class Initialise(smach.State):
	def __init__(self, lib, timeout):
		smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
                self.__controller = lib
                self.__timeout = timeout
  		
	def execute(self,userdata):
                global pub
                #Set Up Publisher for Mission Control Log
                pub = rospy.Publisher('MissionStrings', String)
                str= 'Entered State Initialise'
                pub.publish(str)

                #record initial time
                time_zero = time.time()

                #initialise all_online the sucess criteria
                all_online = False

                #Main loop will run till all online or time_out reached
                while (time.time()-time_zero < self.__timeout) and not(all_online):
                   	all_online = (self.__controller.getArduinoStatus  
                            and self.__controller.getGPSStatus()
                            and self.__controller.getCompassStatus())

                    


               		str= 'gps status = %r' %self.__controller.getGPSStatus()
                	pub.publish(str)
                        rospy.loginfo(str)
                	str='compass status = %r' %self.__controller.getCompassStatus()
                	pub.publish(str)
                        rospy.loginfo(str)                        
                        str='arduino status = %r' %self.__controller.getArduinoStatus()
                	pub.publish(str)
                        rospy.loginfo(str)                        
                	str= 'all online =  = %r' %all_online
                	pub.publish(str)
                        rospy.loginfo(str)                        
                	str='time elapsed = %s s' %(time.time()-time_zero)
                	pub.publish(str)
                        rospy.loginfo(str)                        
                        time.sleep(0.5)

                #if timeout...                
                if all_online == False:    
                    str="One or more critical systems have not come online within the timeout (%ss)" %self.__timeout 
                    rospy.logerr(str)
                    pub.publish(str)          
                    str='Initialise State Aborted' 
                    pub.publish(str)                      
                    return 'aborted'
                
                #Check BatteryVoltage
                voltage=self.__controller.getBatteryVoltage()
                if voltage < 8:
                    str="Initial battery voltage, %sV < 8V" %voltage
                    rospy.logerr(str)  
                    pub.publish(str)       
                    str='Initialise State Preempted' 
                    pub.publish(str)                             
                    return 'preempted'
                else:
                    str="All critical systems have come online within the timeout (%ss)" %self.__timeout
                    rospy.loginfo(str) 
                    str='Initialise State Succeded' 
                    pub.publish(str)  
                    return 'succeeded'
                
                
