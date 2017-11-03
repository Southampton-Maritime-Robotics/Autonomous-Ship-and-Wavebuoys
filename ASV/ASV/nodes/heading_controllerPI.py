#!/usr/bin/python
##############################################################################
#heading_controller.py
#
#Initial code is a simplified subset of heading_controller.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#Completed, checked, modified, corrected by Enrico Anderlini (ea3g09@soton.ac.uk)
#
#Modifications to code
# 21/11/2012	Modified code for ASV	ABP
# 03/02/2013    Finished completion and debugging of the  code, including change
#               of the publishers/subscribers names. Extensive comments added.
# 06/04/2013    Gains modified after model testing. Added a subscriber to the 
#               Boolean published by the circle and zigzag manoeuvres.
#
##############################################################################
#Notes
#
#This code is the heading controller for the ASV. When a heading demand is set
#in the states (e.g. state_goToXY), this node subscribes to that value and 
#obtains a rudder angle demand that publishes to the arduino node. The controller
#is a simple PID feedback controller (digital implementation) that employs the
#current heading value from the compass in order to estimate the current error. 
#The values of the PID constants is based on the actual model tests data (from the zigzag
#manoeuvres in fact - see report).
#In this code, only a PI controller is implemented.
#
################################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import time
import numpy
from ASV.msg import compass
#from ASV.msg import headingd          uncomment to read heading demand during debugging
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import Bool

######################### DEFINE GLOBAL VARIABLES ##############################
### General ###
#    min_int_error  =-10  ##maximum and minimum integral errors in order to avoid an excessively slackish response.
#    max_int_error  = 10  

########################## Rudder PID Controller ###############################
####Setting the Proportional, Integral and Derivative gains####
Pgain      =  0.6292  ##These values have been obtained theoretically - see report
Igain	   =  0.0048
Dgain	   =  0.0
####Setting up the limits to the rudder angle in order to avoid an excessive rudder angle must be avoided, as it would 
#be less efficient due to stall. In addition, the rudder MUST NOT turn by 180 deg!####
rudder_min = -35      
rudder_max =  35

################################################################################
################################################################################

#This function constrains the value of the variable within defined limits-notice the variables order within brackets.
def limits(value, min, max):       
    if value < min:				   
       value = min
    elif value > max:
       value = max
    return value

################################################################################

################################################################################
########## MAIN CONTROL LOOP ###################################################
################################################################################

def main_control_loop():
        ###Defining global variables to be used within the main loop###
    	global heading_request          ##see comment below in function heading_demand_cb
    	global current_heading
    	global controller_onOff
		
	# The following parameter 
	global manoeuvring
		
        #Initialise parmeters       
        controller_onOff = Bool()
        time_previous    = time.time()    
        heading_request  = 0           
        current_heading  = 0  
	error_previous	 = 0            ##used to simplify the derivative term
	int_error 	 = 0  				##initialization
	manoeuvring  = 0

        #Enter main loop
        while not (rospy.is_shutdown() and manoeuvring == 1):
			
            ## Calculate time since last calculation--Defining delta(t)##
            dt = time.time() - time_previous 
			
            #Ensure Demand lies between 0 and 360
            heading_request = (heading_request)%360
            #Calculate Heading Error
            error  = heading_request - current_heading

            #Ensure error lies between -180 and 180			    
            if error <-180:
        	error =   error%360
            if error > 180:
        	error= -(-error%360)

            ### INTEGRAL ###   
            int_error += dt*error	##This has been take from the original code
            # Calculate the integral error 
            #int_error =limits(int_error,min_int_error, max_int_error)  #uncomment for faster response.

            ### DERIVATIVE ###
            ##possible problems at the first time-step. A simple finite-difference,  first-order accurate,  backward scheme is used.##
            der_error = (error-error_previous)/ dt     

            time_previous = time.time()          ##This updates the value of time-zero as that of the previous time-step
            error_previous= error	       	     ##This updates the value of the error_zero to that of the previous time-step
                
                
            ############ALGORITHM TO CALCULATE RUDDER RESPONSE##############
            Pterm = Pgain * error 
            Iterm = Igain * int_error
            Dterm = Dgain * der_error
                
            rudderAngleDemand=Pterm+Iterm+Dterm   
            #Avoid an excessive rudder angle (which stalls, ts being less efficient) or a complete rudder rotation (180deg)
            #in the worst case scenario
            rudder_demand=limits(rudderAngleDemand, rudder_min, rudder_max) 
                
            #####Uncomment the following lines during debugging only#####
            #print'Heading demand:', heading_request
            #print'Current heading:', current_heading
            #print'Rudder angle demand:', rudder_demand    
                 
            #If Heading control is turned on publish rudder demand
            if controller_onOff:
                pub.publish(rudder_demand)
                
            #The following line is to avoid the node to publish values at too high a rate
            rospy.sleep(0.5)


################################################################################
######## END OF Main Control Loop    ###########################################
################################################################################


################################################################################
#The following functions are used for the correct set up of the subscribers-see
#later in the code. It is fundamental to note that the value within brackets and
#the global value must have different names in order to avoid errors.

def heading_demand_cb(heading_demand):          #During debugging, it may be changed with headingd (a message file)            
    global heading_request                      #So as to avoid error where global and local variables have the same name
    heading_request = heading_demand.data       #During debugging, it may be changed with headingd.heading_demand in conjuction with the line above             

def compass_cb(compass):
    global current_heading
    current_heading = compass.heading

def onOff_cb(onOff):
    global controller_onOff
    controller_onOff=onOff.data

#The following call-back function is used to read the Boolean published by the circle and zigzag nodes
def manoeuvring_cb(Manoeuvring):
    global manoeuvring
    manoeuvring=Manoeuvring.data

################################################################################
def shutdown(args=None):
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    
    #for debugging
    print 'entered shutdown'
    
    lib = library_highlevel()
    
    lib.stop() 

################################################################################
######## INITIALISATION ########################################################
################################################################################

if __name__ == '__main__':
    rospy.init_node('Heading_controller')

    ########################SET UP THE SUBSCRIBERS##############################
    rospy.Subscriber('heading_demand', Float32, heading_demand_cb)     #During debugging, it may be changed with , headingd, in conjuction with function above
    rospy.Subscriber('compass_out', compass, compass_cb)
    rospy.Subscriber('Heading_onOFF', Bool, onOff_cb)
    rospy.Subscriber('manoeuvring', Int8, manoeuvring_cb)

    ###########################SET UP THE PUBLISHERS############################
    pub=rospy.Publisher('rudder_demand', Float32)  ##In the original version when working with smach headingd->Float32
    
    ###############Call main_control_loop######################
    rospy.loginfo("Heading controller online")
    main_control_loop()
    
    rospy.on_shutdown(shutdown)