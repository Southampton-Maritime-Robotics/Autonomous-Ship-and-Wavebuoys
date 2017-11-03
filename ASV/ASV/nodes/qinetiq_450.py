#!/usr/bin/python
##############################################################################
#qinetiq_fulllow.py
#
#This code has been created by Enrico Anderlini for the tests at QinetiQ. 
#The code will be crosschecked by Hieu Le.
#
#Modifications to code
# 19/02/2013	Code created from qinetiq.py template
#
##############################################################################
#Notes
#
#The expected model-self propulsion point has been estimated using the 
#Wageningen B-series. Hence, it may be incorrect and some margin has been left
#for this reason.
#
#This code is designed to move the model at about 800 rpm (allowing for 
#difference between ship and model self-propulsion points) at a speed of 
#1.03 m/s (15.5 knots full-scale, operating speed). The model is to operate at
#the same rpm for the whole tank length (about 170 m) in full-loading condition.
#
#This node has been modified with the addition of other 2 possible 
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
            
            #For debugging it is suggested to substitute the following number with 10
            if (time.time()-time_zero)>5 and (time.time()-time_zero)<=120.0:     
                #Hereafter uncomment the desired operational mode
                prop_demand=450                               #rpm, lower than the model self-propulsion point
                #motor_voltage = 0.7                           #percentage of total possible voltage
                #motor_power= 0.7                              #percentage of total possible voltage
                #duty_cycle=170
                
                lib.setProp(prop_demand)  
                #lib.setMotorVoltage(motor_voltage)
                #lib.setMotorPower(motor_power)
                #lib.setDutyCycle(duty_cycle)
                
            elif (time.time()-time_zero)>120.0:
                
                lib.stop()
                return

            
            #This is fundamental to prevent the computer from using 100% CPU.
            #This means the node publishes the demand every 0.50 seconds.
            rospy.sleep(0.50)
            
############################### SHUTDOWN FUNCTION ##############################

def shutdown(args=None):
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    
    #for debugging
    print 'entered shutdown'
    
    lib = library_highlevel()
    
    lib.setRudderAngle(0)
    lib.setMotorTarget(0)
    lib.setMotorSetting(0)
            
################################################################################

#### MAIN FUNCTION ####
if __name__ == '__main__':

    
    qinetiq()
     
    rospy.on_shutdown(shutdown)