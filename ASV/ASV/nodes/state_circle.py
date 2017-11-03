#!/usr/bin/env python
##############################################################################
#state_circle.py
#
#Code developed by Enrico Anderlini (ea3g09@soton.ac.uk) in order to perform
#circle manoeuvres on ASV.
#
#
#Modifications to code
# 12/12/2012	Created code for ASV ABP
# 02/02/2013    Publishers corrected
# 03/02/2013    Extensive comments added
# 14/02/2013    Added possibility to operate with motor voltage, power or 
#               propeller rpm
# 12/03/2013    Added extra if-loop (initial one) so as to read a value for the
#               zero heading; added use of heading controller to maintain 
#               straight course initially
# 06/04/2013	added publisher of a boolean operator to be picked up by the heading
#               controller.
#
##############################################################################
#Notes
#
#This state should be run in order to perform circle manoeuvres in accordance 
#with ITTC. It publishes rudder angle and propeller rpm demands directly to the
#arduino based on the time and current heading read from (subscribed to) the
#compass node. In addition, it also subscribes to the current position from
#gps output.
#
#As a smach state this file must be run from the mission_script.py node.
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

#from library_highlevel import library_highlevel

####Main function of the smach state circle. For more information on smach states
#see http://www.ros.org/wiki/smach####
class circle(smach.State):
    ###Initialising the smach state###
    def __init__(self, lib, prop, angle, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__prop = prop
            self.__angle = angle
            self.__timeout    = timeout
    ###This function contains the code that will actually be executed from the smach state###
    def execute(self,userdata):
            ###Defining global variables###
            global pub
            global prop_demand
            global rudder_demand
            #global motor_voltage
            #global motor_power
            global manoeuvring
            
            time_zero = time.time()
            
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)
            
            str='Entered Circle State at %s' %time_zero
            pub.publish(str)
            rospy.loginfo(str)

            #Getting initial location from position message using the library_highlevel.py functions
            str='Initial Location is X=%.3f, Y=%.3f' %(self.__controller.getX(), self.__controller.getY())
            pub.publish(str)
            rospy.loginfo(str)
           
            ##### Main loop #####  
            ##Entering the main loop only when there are no errors anywhere else in the package##         
            while not rospy.is_shutdown() and (time.time()-time_zero) < self.__timeout and self.__controller.getBackSeatErrorFlag() == 0:
                
                #Hereafter uncomment the desired operational mode
                ##Setting propeller rpm demand using controls set in the library_highlevel.py
                prop_demand = self.__prop
                #motor_voltage = 0.8                           #percentage of total possible voltage
                #motor_power= 0.8                              #percentage of total possible voltage
                self.__controller.setProp(prop_demand)  
                #self.__controller.setMotorVoltage(motor_voltage)
                #self.__controller.setMotorPower(motor_power)  

		manoeuvring = 1
		self.__controller.setManoeuvring(manoeuvring)				
                
                #start the manoeuvre only 10 seconds after a straight run
                if (time.time()-time_zero)<10:
                
                    # Setting the zero heading
                    heading_zero       = self.__controller.getHeading() 
                    
                    #Maintaining the original course during the straight run
                    self.__controller.setHeading(heading_zero)
                
                    ##Setting the rudder angle demand using controls set in the library_highlevel.py
                    rudder_demand = 0
                    self.__controller.setRudderAngle(rudder_demand)

                else:
                    #Starting the circle manoeuvre
                    
                    rudder_demand  = self.__angle  #degrees, starboard. 
                    
                    ##check-uncomment for debugging only
                    #print'Heading_zero :', heading_zero
                    
                    #very empirical approach: this starts considering the heading angle 5 seconds after the start of the manoeuver
                    #in order to avoid being locked by the inertia of the boat.
                    if (time.time()-time_zero)<15:                        
                         self.__controller.setRudderAngle(rudder_demand)  
                         
                         ##check-uncomment for debugging only
                         #print'Time:', (time.time()-time_zero)
                         #print'Rudder angle demand:', rudder_demand
                         
                    else:
                        
                        #When the heading returns close to the original one, end the manoeuvre and return to
                        #a straight run. However, given the sample rate, the relative inaccuracy of the equipment
                        #and the inertia of the model, a relatively large margin has been left.
                        #This value (5) may be corrected after the first tests.
                        while abs(self.__controller.getHeading()-heading_zero)>5:                 
                            self.__controller.setRudderAngle(rudder_demand)
                            ##check-uncomment for debugging only
                            #print'Time:', (time.time()-time_zero)
                            #print'Rudder angle demand:', rudder_demand
                            #print'Current heading:', headingCurrent
                            
                            time.sleep(0.1)
                         
                        else:
                            #Finish manoeuvre and go back to straight run - stopping motor just in case
                            self.__controller.stop()
                            
                            #End of manoeuvre logging
                            str='Final Location is X=%.3f, Y=%.3f' %(self.__controller.getX(), self.__controller.getY())
                            pub.publish(str)
                            rospy.loginfo(str)
                            #uncomment the following lines when debugging
                            #print'Final location is:',str
                            #print'Rudder angle demand:', rudder_demand
                            #print'Time:', time.time()
                            
                            ##When a state is completed it returns an outcome, in this case succeeded
                            return 'succeeded'  
                
                #The following line prevents the node to use 100% CPU
                time.sleep(0.1)
                        
            ##### Exited Main loop either will have timed out or been preempted by backseat driver flag#####
            #This ifloop defines the preempted situation and exits with the appropriate outcome statement.
            if self.__controller.getBackSeatErrorFlag() == 1:
                rospy.logerr("BackSeatDriver Identified Fault") 
                str = 'Backseat Driver Identified Fault circle State preempted'
                pub.publish(str)
                return 'preempted'
            #Time out of the circle manoeuvre-->aborting state
            else:
                rospy.logerr("Go To Waypoint Timed Out")  
                str='circle state timed out, state aborted'
                pub.publish(str)              
                return 'aborted'  
                