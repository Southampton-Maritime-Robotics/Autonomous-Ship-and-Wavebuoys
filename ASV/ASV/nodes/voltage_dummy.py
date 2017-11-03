#!/usr/bin/python
##############################################################################
#voltage_dummy.py
#
#Code by Dr Alexander Brian Phillips
#
#
#Modifications to code
# 21/11/2012	Created code for ASV	ABP
#
#
#
##############################################################################
#Notes
#
#This code publishes dummy values of voltage readings 
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import time
from std_msgs.msg import Float32

def voltage_dummy():
        rospy.init_node('voltage_dummy')
		
		#Define Publishers
        pub = rospy.Publisher('battery_voltage',Float32)
       

        while not rospy.is_shutdown():
			#Define dummy values            
			voltage=12	
			#Publish Dummy Values     
			pub.publish(data=voltage)
			#Sleep
 			rospy.sleep(0.50)
  
if __name__ == '__main__':
	try:
		voltage_dummy()
	except rospy.ROSInterruptException: pass




    

    
    
