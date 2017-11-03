#!/usr/bin/env python

##############################################################################
#zigzag750.py
#
#This code has been created by Enrico Anderlini (ea3g09@soton.ac.uk) for 
#performing zigzag manoeuvring tests.
#
#Modifications to code
#14/03/2013     code created
#06/04/2013		publisher of a boolean operator to be picked up by the heading
#               controller added.
#
##############################################################################
#Notes
#
#This code does not require the running of the smach state machine. It performs
#an initial straight run of 10 seconds and subsequently starts a zigzag manoeuvre.
#At the moment it starts turning to starboard but this may be modified easily.
#Note that while this node is running, the heading controller will be turned off.
#At the moment the node is performing 3 bends only - this may be easily modified.
#
##############################################################################

import roslib; roslib.load_manifest('ASV')
import rospy
import numpy
#import library highlevel
from library_highlevel            import library_highlevel
import time
from std_msgs.msg import String
from ASV.msg import compass
from ASV.msg import position

###Defining global variables###
global pub
global prop_demand
global rudder_demand
global manoeuvring

def zigzag():
    #Initialising the node
    rospy.init_node('Zigzag')

    #Set Up Publisher for Mission Control Log
    pub = rospy.Publisher('MissionStrings', String)

    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    
    time_zero = time.time()
    
    str='Entered ZigZag State at %s' %time_zero
    pub.publish(str)
    rospy.loginfo(str)


    str='Initial Location is X=%.3f, Y=%.3f' %(lib.getX(), lib.getY())
    pub.publish(str)
    rospy.loginfo(str)
    
   
    ########################## Main loop ###############################    
    ##### Enter the main loop only when there are no errors#####       
    while (not rospy.is_shutdown()):
        
        #Hereafter uncomment the desired operational mode
        #Setting the propeller demand#
        prop_demand = 750                      #This may need changing: it is the rpm value that yields the desired model speed
        #motor_voltage = 0.8                           #percentage of total possible voltage
        #motor_power= 0.8                              #percentage of total possible voltage
        lib.setProp(prop_demand)         ##or motor voltage after the first experimental tests with the motor.
        #lib.setMotorVoltage(motor_voltage)
        #lib.setMotorPower(motor_power) 
		
	manoeuvring = 1
	lib.setManoeuvring(manoeuvring)
        
        #start the manoeuvre only 10 seconds after a straight run
        if (time.time()-time_zero)<10:
            
            rudder_demand = 0
            lib.setRudderAngle(rudder_demand)
            
            heading_zero = lib.getHeading()
           
        else:
            #Start the zig-zag manoeuvre to starboard - Starboard means the heading increases with time - remember this for 
            #manoeuvre to port
            rudder_demand= 20
            lib.setRudderAngle(rudder_demand)
            
            while (rudder_demand-(lib.getHeading()-heading_zero))>0:
                rudder_demand  = 20  #degrees, starboard. Change with 10 for 10-10 instead of 20-20 zig-zag manoeuvre.
                lib.setRudderAngle(rudder_demand)                
                time.sleep(0.1)
                
            #Once the current heading is greater than the rudder angle (overshoot), 
            #turn to the other side 
            else:
                
                while ((lib.getHeading()-heading_zero)-rudder_demand) >0:
                    rudder_demand = -20 #degrees, to port. May be changed with 10, see above
                    lib.setRudderAngle(rudder_demand)
                    time.sleep(0.1)
                               
                #Exit the manoeuvre with success
                else:
                    rudder_demand= 20
                    lib.setRudderAngle(rudder_demand)
                    
                    while (rudder_demand-(lib.getHeading()-heading_zero))>0:
                        rudder_demand  = 20  #degrees, starboard. Change with 10 for 10-10 instead of 20-20 zig-zag manoeuvre.
                        lib.setRudderAngle(rudder_demand)
                
                        time.sleep(0.1)
                        
                    else:
                        
                        while ((lib.getHeading()-heading_zero)-rudder_demand) >0:
                            rudder_demand = -20 #degrees, to port. May be changed with 10, see above
                            lib.setRudderAngle(rudder_demand)
                            time.sleep(0.1)
                        
                        else:
                                               
                            lib.stop() 
                
        #The following line prevents the node from using 100% CPU
        time.sleep(0.1)
        


def shutdown(args=None):
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    
    #for debugging
    print 'entered shutdown'
    
    lib = library_highlevel()
    
    lib.stop()
    
            
################################################################################

#### MAIN FUNCTION ####
if __name__ == '__main__':

    
    zigzag()
     
    rospy.on_shutdown(shutdown)
    
 