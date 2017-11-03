#!/usr/bin/python
##############################################################################
#arduino_dummy.py
#
#Initial code is a simplified subset of compass_dummy.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#
#Modifications to code
#21/11/2012	Modified code for ASV	ABP
#31/01/2013   Modified for actual interactions with the Arduino unit
#
##############################################################################
#Notes
#
#This code publishes dummy values of prop_rpm and rudder_angle 
#and subscribes to rudder_demand and prop_demand 
#
#a def will need to be added to open the serial port, close the serial port, main control loop needs to then read/write to the serial port.
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import time
from std_msgs.msg import Float32
from ASV.msg import status

global rudder_demand

def arduino_dummy():
        rospy.init_node('arduino_dummy')
        
        #The following line must be uncommented for the debugging of state_circle.py and state_zigzag.py
        time_zero = time.time()                      
		
        #Define Publishers
        pub = rospy.Publisher('rudder_demand', Float32)

       
	
        #Publish compass status as true!
        pubStatus = rospy.Publisher('status', status)



        while not rospy.is_shutdown():
	    #Define dummy compass values            
            #The following line must be commented when the file is used for the 
            #debugging of state_circle.py and state_zigzag.py.
            #heading=320.0
            rudder_demand=-50  
            
            
            NodeOn=True
            #Publish Dummy Values     
            pub.publish(rudder_demand)
            pubStatus.publish(nodeID = 7, status = NodeOn)
            
            #Sleep
            rospy.sleep(0.50)
  
if __name__ == '__main__':
	try:
            arduino_dummy()
	except rospy.ROSInterruptException: pass

    
