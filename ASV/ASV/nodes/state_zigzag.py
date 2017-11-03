#!/usr/bin/env python
##############################################################################
#state_zigzag.py
#
#Code developed by Enrico Anderlini (ea3g09@soton.ac.uk) in order to perform
#zig-zag manoeuvres on ASV.
#
#
#Modifications to code
# 12/12/2012	Created code for ASV ABP
# 02/02/2013    Publishers and their values corrected
# 04/02/2013    Extensive comments added to the code
# 14/02/2013    Added possibility to select operational mode: motor voltage,
#               power or propeller rpm
# 27/02/2013    Simplified code (if-else loops), added rudder and rpm setting
#               to zero at the end of the simulation (as the arduinos don't do 
#               it by default.
# 12/03/2013    Added heading zero value as an offset value so that the 
#               manoeuvre can be performed for any initial heading; added use
#               of heading controller to maintain straight course initially
#
##############################################################################
#Notes
#
#This state should be run in order to perform zig-zag manoeuvres in accordance 
#with ITTC. It publishes rudder angle and propeller rpm demands directly to the
#arduino based on the time and current heading read from (subscribed to) the
#compass node. In addition, it also subscribes to the current position from
#gps output.
#
#As a smach state this file must be run from the mission_script.py node.
#For more information on smach states see http://www.ros.org/wiki/smach.
#
#All self ("getter" and "setter") functions are defined in the 
#library_highlevel.py node.
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import numpy
import smach
import smach_ros
import time
from std_msgs.msg import String
from ASV.msg import compass
from ASV.msg import position

###########################SMACH STATE ZIGZAG##################################

class zigzag(smach.State):
    ###Initialising the smach state###
    def __init__(self, lib, prop, angle, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__prop = prop
            self.__angle = angle
            self.__timeout       = timeout
    ###This function contains the code that will actually be executed from the smach state###        
    def execute(self,userdata):
            ###Defining global variables###
            global pub
            global prop_demand
            global rudder_demand
            #global motor_voltage
            #global motor_power
            global manoeuvring
            
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)

            time_zero = time.time()
            
            str='Entered ZigZag State at %s' %time_zero
            pub.publish(str)
            rospy.loginfo(str)

  
            str='Initial Location is X=%.3f, Y=%.3f' %(self.__controller.getX(), self.__controller.getY())
            pub.publish(str)
            rospy.loginfo(str)
	    
            
           
            ########################## Main loop ###############################    
            ##### Enter the main loop only when there are no errors#####       
            while not rospy.is_shutdown() and (time.time()-time_zero) < self.__timeout and self.__controller.getBackSeatErrorFlag() == 0:
                
                print "Don't worry, I just had a couple of pints"   #Please, remove this before submission
                
                #Hereafter uncomment the desired operational mode
                #Setting the propeller demand#
                prop_demand = self.__prop                      
                #motor_voltage = 0.8                           #percentage of total possible voltage
                #motor_power= 0.8                              #percentage of total possible voltage
                self.__controller.setProp(prop_demand)         ##or motor voltage after the first experimental tests with the motor.
                #self.__controller.setMotorVoltage(motor_voltage)
                #self.__controller.setMotorPower(motor_power) 
                
                manoeuvring = 1
                self.__controller.setManoeuvring(manoeuvring)
                
                #start the manoeuvre only 10 seconds after a straight run
                if (time.time()-time_zero)<10:
            
                    rudder_demand = 0
                    self.__controller.setRudderAngle(rudder_demand)
                    
                    #Reading the zero heading which will be used as an offset value
                    heading_zero = self.__controller.getHeading()
                    
                    #Maintaining the original course during the straight run
                    self.__controller.setHeading(heading_zero)
                   
                else:
                    #Start the zig-zag manoeuvre to starboard
                    rudder_demand= sef.__angle
                    self.__controller.setRudderAngle(rudder_demand)
                    
                    #Removing the offset value for the heading so that it can be subtracted from the rudder angle
                    while (rudder_demand-(self.__controller.getHeading()-heading_zero))>0:
                        rudder_demand  = sef.__angle  #degrees, starboard.
                        self.__controller.setRudderAngle(rudder_demand)
                        ##check-uncomment only during debugging
                        #print'Time:', (time.time()-time_zero)
                        #print'Rudder angle demand:', rudder_demand
                        #print'Current heading:', headingCurrent
                        
                        time.sleep(0.1)
                        
                    #Once the current heading is greater than the rudder angle (overshoot), 
                    #turn to the other side 
                    else:

                        while ((self.__controller.getHeading()-heading_zero)-rudder_demand) >0:
                            rudder_demand = -sef.__angle  #degrees, to port
                            self.__controller.setRudderAngle(rudder_demand)
                            ##check-uncomment only during debugging
                            #print'Time:', (time.time()-time_zero)
                            #print'Rudder angle demand:', rudder_demand
                            #print'Current heading:', headingCurrent
                            
                            time.sleep(0.1)
                            
                        #Exit the manoeuvre with success
                        else:
                    
							while (rudder_demand-(lib.getHeading()-heading_zero))>0:
								rudder_demand  = sef.__angle  #degrees, starboard. 
								lib.setRudderAngle(rudder_demand)
                
								time.sleep(0.1)
                        
							else:
                        
								while ((lib.getHeading()-heading_zero)-rudder_demand) >0:
									rudder_demand = -sef.__angle #degrees, to port.
									lib.setRudderAngle(rudder_demand)
									time.sleep(0.1)
                        
                                                                else:
                                     
                                                                    self.__controller.stop() 
                                                                    return 'succeeded'
                        
                #The following line prevents the node from using 100% CPU
                time.sleep(0.1)
                        
            ##### Exited Main loop either will have timed out or been preempted by backseat driver flag#####
            if self.__controller.getBackSeatErrorFlag() == 1:
                rospy.logerr("BackSeatDriver Identified Fault") 
                str = 'Backseat Driver Identified Fault zigzag State preempted'
                pub.publish(str)
                return 'preempted'
            else:
                rospy.logerr("Go To Waypoint Timed Out")  
                str='zigzag state timed out, state aborted'
                pub.publish(str)              
                return 'aborted'  
                