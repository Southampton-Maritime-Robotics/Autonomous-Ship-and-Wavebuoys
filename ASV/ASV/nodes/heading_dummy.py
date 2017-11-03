#!/usr/bin/python
##############################################################################
#heading_dummy.py
#
#This code is an adaption of compass_dummy.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#It has been created by Enrico Anderlini for debugging heading_controller.py 
#(after finding a strange beahvaiour).
#
#Modifications to code
#30/01/2013	Modified code for ASV	ABP
#
#
#
##############################################################################
#Notes
#
#This code publishes dummy values of heading demand
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import time
from std_msgs.msg import Float32
from ASV.msg import headingd
from ASV.msg import status

def heading_dummy():
        rospy.init_node('heading_dummy')
        
        
		
        #Define Publishers
        #pub = rospy.Publisher('heading_demand', Float32)  #It may be changed with headingd
        #This is fundamental in order for it to work with heading_controller.py 

        
        #Publish heading demand status as true!
        pubStatus = rospy.Publisher('status', status)

        time_zero =time.time()


        while not rospy.is_shutdown():
	    #Define dummy heading demand value  
            if (time.time()-time_zero)<15:
                    
                heading_demand=300.0 
            
                print'heading demand:', heading_demand
            
                NodeOn=True
                #Publish Dummy Values     
                pub.publish(heading_demand)             #=heading_demand
                pubStatus.publish(nodeID = 6, status = NodeOn)
                
                #Sleep
                rospy.sleep(0.50)
                
            elif (time.time()-time_zero)<35:
                
                heading_demand=150.0 
            
                print'heading demand:', heading_demand
            
                NodeOn=True
                #Publish Dummy Values     
                pub.publish(heading_demand)             #=heading_demand
                pubStatus.publish(nodeID = 6, status = NodeOn)
                
                #Sleep
                rospy.sleep(0.50)
            
            else:
                heading_demand=303
                print'heading demand:', heading_demand
            
                NodeOn=True
                #Publish Dummy Values     
                pub.publish(heading_demand)             #=heading_demand
                pubStatus.publish(nodeID = 6, status = NodeOn)
                
                #Sleep
                rospy.sleep(0.50)
                

  
if __name__ == '__main__':
        
        pub = rospy.Publisher('heading_demand', Float32)
        
	try:
            heading_dummy()
	except rospy.ROSInterruptException: pass




    

    