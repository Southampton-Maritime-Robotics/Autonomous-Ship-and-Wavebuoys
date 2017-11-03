#!/usr/bin/env python
##############################################################################
#state_goToXY.py.py
#
#Initial code is a simplified subset of state_goToXYZ.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#Code modified, corrected and filled in by Enrico Anderlini (ea3g09@soton.ac.uk)
#
#Modifications to code
# 21/11/2012	Modified code for ASV	ABP
# 11/12/2012    Modified in order to fill in the gaps and adapt the code to the
#               ASV
# 04/02/2013    Extensive comments added
# 14/02/2013    Added the possibility to select operational mode: motor voltage,
#               power or propeller rpm
#
#
##############################################################################
#Notes
#
#This state should be run in order to move the  model towards specified points.
#It is a smach state, run from mission_script.py. For more information on smach 
#states see http://www.ros.org/wiki/smach.
#In the original version a track-follow algorithm based on Mcphail and Pebody, 
#Navigation and Control of an Autonomous Underwater Vehicle, Underwater 
#Technology, Vol 23 No,1 1998 was employed. 
#In this case a simplified algorithm is used that makes use of the fact that the 
#tests should be performed mainly with one model speed. This could be improved
#by accounting for the lake dimensions.
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

#############################SMACH STATE GOTOXY################################
class GoToXY(smach.State):
    ####Initialising the smach state####
    def __init__(self, lib, WpXnew, WpYnew, XYtolerance, timeout):
            smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
            self.__controller = lib
            self.__WpXnew = WpXnew
            self.__WpYnew = WpYnew
            self.__XYtolerance   = XYtolerance
            self.__timeout       = timeout
    ####Main Execute function####
    def execute(self,userdata):
            global pub
            #Set Up Publisher for Mission Control Log
            pub = rospy.Publisher('MissionStrings', String)

            time_zero = time.time()
            
            str='Entered GoToXY State at %s' %time_zero
            pub.publish(str)
            rospy.loginfo(str)
            str='Desired Position X=%.3f m, Y=%.3f m' %(self.__WpXnew, self.__WpYnew) 
            pub.publish(str)
            rospy.loginfo(str)
	    
            str='Initial Location is X=%.3f, Y=%.3f' %(self.__controller.getX(), self.__controller.getY())
            pub.publish(str)
            rospy.loginfo(str)
	    

            dx = self.__WpXnew - self.__controller.getX()
            dy = self.__WpYnew - self.__controller.getY()          
            Range = (dx**2 + dy**2)**0.5
            str = "Initial Range to Waypoint %.3f" %Range
            pub.publish(str)
            rospy.loginfo(str)
            
           
            ##### Main loop #####           
            while not rospy.is_shutdown() and (time.time()-time_zero) < self.__timeout and self.__controller.getBackSeatErrorFlag() == 0:
                loop_time_zero = time.time()
                
                #Get Current Position in X and Y
                X=self.__controller.getX()
                Y=self.__controller.getY()
                heading=self.__controller.getHeading()
				
                #Calculate Range To Target
                dx = self.__WpXnew - self.__controller.getX()
                dy = self.__WpYnew - self.__controller.getY()          
                Range = (dx**2 + dy**2)**0.5
                Bearing = numpy.arctan2(dx,dy) 
                
                str='Range=%s, Bearing=%s' %(Range, Bearing)
                rospy.loginfo(str)
	
                #Check if Range to target is less than xytolerance, set in the launch file and published from the BackSeatDriver.py node
                if Range< self.__XYtolerance:
                    # Additions required because thee arduino do not shut down by default
                    
                    self.__controller.stop()  
                    
                    str = 'WayPoint Reached'
                    pub.publish(str)
                    return 'succeeded'
                ############## MAIN ALGORITHM #################
                else:
                        #The following if loop may be commented and substituted
                        #with the original code-see below- if found unsatisfactory.
			##The heading is zero towards north, hence if aligned with the y-axis.
                        if dx>=0:
                            headingDemand = numpy.pi/2- Bearing    #This is valid also for dy<0 because the double negative sign cancels out
                        else:
                            headingDemand = 3*numpy.pi/2-Bearing
                            
			##The original track-follow alogirthm, suitably modified according to the variables used here is as follows. It 
                        ##can be uncommented and the above loop commented if it is desired to use this code instead.
                        ##if dy>0:
                        ##    dummy=dx/dy
                        ##    headingDemand=2*numpy.pi+numpy.arctan(dummy)
                        ##    headingDemand=headingDemand%(2*numpy.pi)
                        ##elif dy<0:
                        ##    dummy=dx/dy
                        ##    headingDemand=numpy.pi+numpy.arctan(dummy)
                        ##elif (dy==0 and dx>0):
                        ##    headingDemand=numpy.pi
                        ##else:
                        ##    headingDemand=3/4*numpy.pi
                        ##I think the two algorithms are in fact equal, with the difference that mine uses dy/dx and the original one dx/dy

                        #Convert  the heading demand into degrees
                        headingDemand=headingDemand/numpy.pi*180
                        
                        #The following lines ensure that the heading error is contained within 360 degrees-avoidance of negative angles
                        headingError=headingDemand-heading   
                        if headingError <-180:
                            headingError =   headingError%360
                        if headingError > 180:
                            headingError= -(-headingError%360)
                        ##The above algorithm ensures that the heading error is contained within 360 degrees.
                        
                        
                        ##The following algorithm is very basic: it makes use of the fact that the model should be operated mostly at the operational
                        ##speed, thus setting smaller power if the difference in heading is very large and increasing the power to the one corresponding
                        ##to the operational speed once the error has decreased to within 45 deg. This avoids excessive turning circles for very large
                        ##heading errors.
                        if abs(headingError) == 180:  ##degrees
                            propDemand = 400           ##rpm, very low (considering the tremendously low operational J)
                            #motor_voltage = 0.8                           #percentage of total possible voltage
                            #motor_power= 0.8                              #percentage of total possible voltage
                        elif abs(headingError)<180 and abs(headingError)>=90:
                            propDemand = 500           ##rpm
                            #motor_voltage = 0.8                           #percentage of total possible voltage
                            #motor_power= 0.8                              #percentage of total possible voltage
                        elif abs(headingError)<90 and abs(headingError)>=45:
                            propDemand = 800           ##rpm
                            #motor_voltage = 0.8                           #percentage of total possible voltage
                            #motor_power= 0.8                              #percentage of total possible voltage
                        else:
                            propDemand = 1000          ##rpm, corresponding to full operational speed
                            #motor_voltage = 0.8                           #percentage of total possible voltage
                            #motor_power= 0.8                              #percentage of total possible voltage
								
                        #Set new Demands - uncomment the desired operational mode
                        self.__controller.setProp(propDemand)  
                        #self.__controller.setMotorVoltage(motor_voltage)
                        #self.__controller.setMotorPower(motor_power) 
			self.__controller.setHeading(headingDemand)   #or self.__controller.setRudderAngle(rudderAngleDemand)


               
                #Only want to update demands every 0.5s hence following wait statements
                dt = time.time() - loop_time_zero
                while dt < 0.5:
                        dt = time.time() - loop_time_zero

               
            ##### Exited Main loop either will have timed out or been preempted by backseat driver flag#####

            if self.__controller.getBackSeatErrorFlag() == 1:
                rospy.logerr("BackSeatDriver Identified Fault") 
                str = 'Backseat Driver Identified Fault GoToXY State preempted'
                pub.publish(str)
                return 'preempted'
            else:
                rospy.logerr("Go To Waypoint Timed Out")  
                str='GoToXYZ state timed out, state aborted'
                pub.publish(str)              
                return 'aborted'  
		

    
