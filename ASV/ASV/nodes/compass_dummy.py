#!/usr/bin/python
##############################################################################
#compass_dummy.py
#
#Initial code is a simplified subset of compass_dummy.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#
#Modifications to code
# 21/11/2012	Modified code for ASV	ABP
#
#
##############################################################################
#Notes
#
#This code publishes dummy values of compass readings 
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import time
from ASV.msg import compass
from ASV.msg import status

def compass_dummy():
        rospy.init_node('compass_dummy')
        
        #The following line must be uncommented for the debugging of state_circle.py and state_zigzag.py
        time_zero = time.time()                      
		
        #Define Publishers
        pub = rospy.Publisher('compass_out',compass)

       
	
        #Publish compass status as true!
        pubStatus = rospy.Publisher('status', status)



        while not rospy.is_shutdown():
	    #Define dummy compass values            
            #The following line must be commented when the file is used for the 
            #debugging of state_circle.py and state_zigzag.py.
            #heading=320.0
            pitch=-5.0
            roll=1.0
            temperature=15.0		
            m=10.0
            mx=5
            my=6
            mz=7
            a=0
            ax=2
            ay=5
            az=5
            prop = 12.0
            demand = 50.0   
            
            ####################################################################
            #The following loop must be uncommented when the file is used for the
            #debugging of state_circle.py and state_zigzag.py.
            if (time.time()-time_zero)<10:
                heading=320.0
            elif (time.time()-time_zero)<20:
                heading=330
            elif (time.time()-time_zero)<25:
                heading=341
            elif (time.time()-time_zero)<30:
                heading=305
            else:
                heading=300
            ####################################################################
            
            print'heading:', heading
            
            NodeOn=True
            #Publish Dummy Values     
            pub.publish(heading=heading,pitch=pitch,roll=roll,temperature=temperature,m=m,mx=mx,my=my,mz=mz,a=a,ax=ax,ay=ay,az=az)
            pubStatus.publish(nodeID = 3, status = NodeOn)
            
            #Sleep
            rospy.sleep(0.50)
  
if __name__ == '__main__':
	try:
            compass_dummy()
	except rospy.ROSInterruptException: pass




    

    
    
