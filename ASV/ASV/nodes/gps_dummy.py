#!/usr/bin/python
##############################################################################
#gps_dummy.py
#
#Initial code is a simplified subset of compass_dummy.py from DelphinROSv2
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
#This code publishes dummy values of gps readings 
#X is positive east!!
#Y is positive north!!
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import time
from ASV.msg import position
from ASV.msg import status

def gps_dummy():
        rospy.init_node('gps_dummy')
        
		
        #Define Publishers
        pub = rospy.Publisher('position',position)

       
	
        #Publish compass status as true!
        pubStatus = rospy.Publisher('status', status)
        
        X=0

        while not rospy.is_shutdown():
	    #Define dummy position values            
            X=X+0.1
            Y=0
            lat=50.93642234975332
            long=-1.507850997377956
            speed=0.1
            ValidGPSfix=True
            
            print'X', X
            print'Y', Y
            print'speed',speed
            
            #Publish Dummy Values     
            pub.publish(X=X, Y=Y,speed=speed, lat=lat,long=long, ValidGPSfix=ValidGPSfix)
            
            NodeOn=True
            pubStatus.publish(nodeID = 2, status = NodeOn)
            #Sleep
            rospy.sleep(0.50)
  
if __name__ == '__main__':
	try:
            gps_dummy()
	except rospy.ROSInterruptException: pass




    

    
    
