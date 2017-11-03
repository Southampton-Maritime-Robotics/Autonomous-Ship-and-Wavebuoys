#!/usr/bin/env python

##############################################################################
#circle30.py
#
#This code has been created by Enrico Anderlini (ea3g09@soton.ac.uk) for 
#performing circle manoeuvring tests.
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
#an initial straight run of 10 seconds and subsequently starts a cirle manoeuvre.
#At the moment it is turning to starboard only but this may be modified easily.
#Note that while this node is running, the heading controller will be turned off.
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

def circle():
    #Initialising the node
    rospy.init_node('Circle')

    #Set Up Publisher for Mission Control Log
    pub = rospy.Publisher('MissionStrings', String)

    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    
    time_zero = time.time()
            
    str='Entered Circle State at %s' %time_zero
    pub.publish(str)
    rospy.loginfo(str)

    #Getting initial location from position message using the library_highlevel.py functions
    str='Initial Location is X=%.3f, Y=%.3f' %(lib.getX(), lib.getY())
    pub.publish(str)
    rospy.loginfo(str)
            
    
    
    ##### Main loop #####  
    ##Entering the main loop only when there are no errors anywhere else in the package##         
    while (not rospy.is_shutdown()):
                
            #Hereafter uncomment the desired operational mode
            ##Setting propeller rpm demand using controls set in the library_highlevel.py
            prop_demand = 1100
            #motor_voltage = 0.8                           #percentage of total possible voltage
            #motor_power= 0.8                              #percentage of total possible voltage
            lib.setProp(prop_demand)  
            #lib.setMotorVoltage(motor_voltage)
            #lib.setMotorPower(motor_power) 
            manoeuvring = 1
            lib.setManoeuvring(manoeuvring)
            
            #start the manoeuvre only 10 seconds after a straight run
            if (time.time()-time_zero)<10:
                
                # Setting the zero heading
                heading_zero       = lib.getHeading() 
                
                ##Setting the rudder angle demand using controls set in the library_highlevel.py
                rudder_demand = 0
                lib.setRudderAngle(rudder_demand)                 
                   
            else:
                #Starting the circle manoeuvre
                
                rudder_demand  = 30  #degrees, starboard. Change with any other value for other rudder angles.
                    
                #very empirical approach: this starts considering the heading angle 5 seconds after the start of the manoeuver
                #in order to avoid being locked by the inertia of the boat.
                if (time.time()-time_zero)<15:                        
                    lib.setRudderAngle(rudder_demand)  
                         
                    #check-uncomment for debugging only
                    #print'Time:', (time.time()-time_zero)
                    #print'Rudder angle demand:', rudder_demand
                         
                else:
                        
                    #When the heading returns close to the original one, end the manoeuvre and return to
                    #a straight run. However, given the sample rate, the relative inaccuracy of the equipment
                    #and the inertia of the model, a relatively large margin has been left.
                    #This value (5) may be corrected after the first tests.
                    while abs(lib.getHeading()-heading_zero)>5:                 
                        lib.setRudderAngle(rudder_demand)
                        ##check-uncomment for debugging only
                        #print'Time:', (time.time()-time_zero)
                        #print'Rudder angle demand:', rudder_demand
                        #print'heading_zero', heading_zero
                        #print'current heading', lib.getHeading()
                        #print'Current heading:', headingCurrent
                        
                        time.sleep(0.1)
                         
                    else:
                        #Finish manoeuvre and go back to straight run - stopping motor just in case
                        lib.stop()
                        #print 'finishing manoeuvre'
                        #print'heading_zero', heading_zero
                        #print'current heading', lib.getHeading()
                            
                        #End of manoeuvre logging
                        str='Final Location is X=%.3f, Y=%.3f' %(lib.getX(), lib.getY())
                        pub.publish(str)
                        rospy.loginfo(str)
                        #uncomment the following lines when debugging
                        #print'Final location is:',str
                        #print'Rudder angle demand:', rudder_demand
                        #print'Time:', time.time() 
                
            #The following line prevents the node to use 100% CPU
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

    
    circle()
     
    rospy.on_shutdown(shutdown)
    
    