#!/usr/bin/python
##############################################################################
#qinetiq.py
#
#This code has been created by Enrico Anderlini for the tests at QinetiQ. 
#The code will be crosschecked by Hieu Le
#
#Modifications to code
#05/02/2013	Code created 
#14/02/2013     Added the possibility to select three operational modes: motor
#               voltage, power and propeller rpm. 
#15/02/2013     Substituted publishers & subscribers with function calls from 
#               the library_highlevel.py node
#19/02/2013     added duty cycle stting possibility
#
##############################################################################
#Notes
#
#This code publishes three sets of propeller rpm values, each one kept constant
#over a time interval. This time interval may be subjected to change. In order
#to obtain it, it has been considered that three propeller rpm per run can be
#measured at QinetiQ. The first prop rpm must lower than the expected self-
#propulsion point, the second one close to it and the last one higher than it.
#At the moment the time intervals have been calculated by considering a 170 m
#(this allows for the initial acceleration part) total run length at a speed
#of 1.026 m/s, corresponding to the full-scale ship operational speed. The time
#intervals are expressed in seconds. At the moment  the model self-propulsion
#point is expected  to occur at 1050 rpm. These values can thus be subjected to
#change once more correct values are known (say, after the initial tests).
#Obviously this node must be run in conjunction with the logger.py.
#
#This node has been modified (see above) with the addition of other 2 possible 
#operational modes (motor voltage and power). In order to simplify this process
#the "setter" function defined in the library_highlevel.py node have been used
#instead of the original subscribers.
#In addition, the publishers and subscribers have been removed and use has been
#made of the library_highlevel.py node instead.
#
#This code could be substituted with a smach state, to be included within the
#node mission_script.py, but that process has been considered to be 
#overcomplicated.
#
#This code is actually a template. For actual files to be used see other qinetiq
#nodes.
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import String
from ASV.msg import status

#import library highlevel
from library_highlevel            import library_highlevel

global prop_demand
#global motor_voltage
#global motor_power
#global duty_cycle

def qinetiq():
        #Initialising the node
        rospy.init_node('QinetiQ')
        
        # Define an instance of highlevelcontrollibrary to pass to all action servers
        lib = library_highlevel()

        #Publish QinetiQ status as true!
        pubStatus = rospy.Publisher('status', status)
        
        #Setting the zero time
        time_zero = time.time()
        
        #Publish the propeller rpm demand only when the node is not shutdown
        while not rospy.is_shutdown():
            
            #Publish the status of the node: online
            NodeOn=True
            pubStatus.publish(nodeID = 3, status = NodeOn)
            
            #For debugging it is suggested to substitute the following number with 5 or 10
            if (time.time()-time_zero)<10:                   #55.2
                #Hereafter uncomment the desired operational mode
                prop_demand=900           #rpm, lower than the model self propulsion point
                #motor_voltage = 0.8                           #percentage of total possible voltage
                #motor_power= 0.8                              #percentage of total possible voltage
                #duty_cycle=170
                
                lib.setProp(prop_demand)  
                #lib.setMotorVoltage(motor_voltage)
                #lib.setMotorPower(motor_power)
                #lib.setDutyCycle(duty_cycle) 
                
            #For debugging it is suggested to substitute the following number with 10 or 20
            elif (time.time()-time_zero)<20:                #110.5
                #Hereafter uncomment the desired operational mode
                prop_demand=1050           #rpm, lower than the model self propulsion point
                #motor_voltage = 0.9                           #percentage of total possible voltage
                #motor_power= 0.9                              #percentage of total possible voltage
                #duty_cycle=170
                
                lib.setProp(prop_demand)   
                #lib.setMotorVoltage(motor_voltage)
                #lib.setMotorPower(motor_power) 
                #lib.setDutyCycle(duty_cycle)
                
            #For debugging it is suggested to substitute the following number with 15 or 30    
            elif (time.time()-time_zero)<30:                 #165.7
                #Hereafter uncomment the desired operational mode
                prop_demand=1100           #rpm, lower than the model self propulsion point
                #motor_voltage = 0.95                           #percentage of total possible voltage
                #motor_power= 0.95                              #percentage of total possible voltage
                #duty_cycle=170
                
                lib.setProp(prop_demand)  
                #lib.setMotorVoltage(motor_voltage)
                #lib.setMotorPower(motor_power) 
                #lib.setDutyCycle(duty_cycle)
            
            #For debugging it is suggested to substitute the following number with 20 or 35    
            elif (time.time()-time_zero)<35:   #170.7        #Extra time, for smooth shutting down of the motor
                #Hereafter uncomment the desired operational mode
                prop_demand=500           #rpm, lower than the model self propulsion point
                #motor_voltage = 0.4                           #percentage of total possible voltage
                #motor_power= 0.4                              #percentage of total possible voltage
                #duty_cycle=170
                
                lib.setProp(prop_demand)   
                #lib.setMotorVoltage(motor_voltage)
                #lib.setMotorPower(motor_power)
                #lib.setDutyCycle(duty_cycle)
                
            else:
                return
            
            #This is fundamental to prevent the computer from using 100% CPU
            rospy.sleep(0.50)
            
################################################################################

#### MAIN FUNCTION ####
if __name__ == '__main__':
        
    try:
        qinetiq()
    except rospy.ROSInterruptException: pass 