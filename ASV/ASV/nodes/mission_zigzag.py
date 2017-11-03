#!/usr/bin/env python
##############################################################################
#mission_script.py
#
#Initial code is a simplified subset of mission_script*.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#States zigzag and circle added by Enrico Anderlini (ea3g09@soton.ac.uk)
#
#Modifications to code
# 21/11/2012	Modified code for ASV	ABP
# 05/02/2013    states zigzag and circle added
#
#
##############################################################################
#Notes
#
#This code generates the state machine/mission that the ASV will complete
#For more information on smach states see http://www.ros.org/wiki/smach, in 
#particular the tutorial section.
#
#X is +ve east
#Y is +ve North
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import smach
import smach_ros
import time

#import library highlevel
from library_highlevel            import library_highlevel

#Import building block states
from state_initialise             import Initialise
from state_stop                   import Stop
from state_goToXY                 import GoToXY
from state_circle                 import circle
from state_zigzag                 import zigzag

#Import messages
from std_msgs.msg import String

def main():

    rospy.init_node('smach_example_state_machine')
    
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    
    #Set Up Publisher for Mission Control Log
    pub = rospy.Publisher('MissionStrings', String)
    

    #Read back seat driver Settings and publish these values are all set in the launch file  
    overPitch = rospy.get_param('over-pitch')    
    str = 'Backseat Driver Parameter: over pitch set to %s deg' %(overPitch)
    pub.publish(str) 
    rospy.loginfo(str)
    time.sleep(0.1)    
    overRoll = rospy.get_param('over-roll')
    str = 'Backseat Driver Parameter: over roll set to %s deg' %(overRoll)
    pub.publish(str) 
    rospy.loginfo(str)
    time.sleep(0.1)         
    maxInternalTemp = rospy.get_param('max-internal-temp')  
    str = 'Backseat Driver Parameter: max internal temperature %s deg' %(maxInternalTemp)
    pub.publish(str)
    rospy.loginfo(str)
    time.sleep(0.1)
    minMotorVoltage = rospy.get_param('min-battery-voltage')
    str = 'Backseat Driver Parameter: min battery voltage %s V' %(minMotorVoltage)
    pub.publish(str) 
    rospy.loginfo(str)
    time.sleep(0.1) 
    missionTimeout = rospy.get_param('mission-timeout') 
    str = 'Backseat Driver Parameter: mission-timeout %s min' %(missionTimeout)
    pub.publish(str) 
    rospy.loginfo(str) 
    
    #Allow system to come online
    time.sleep(5)  
    
    
    #Display and Record Starting Location latOrigin and longOrigin must be defined in the launch file 
    latOrigin = rospy.get_param('lat_orig')
    longOrigin = rospy.get_param('long_orig')
    str = 'Origin Loaded, Longitude=%s Latitude=%s' %(longOrigin,latOrigin)
    pub.publish(str)
    rospy.loginfo(str)
    X=lib.getX()
    Y=lib.getY()
    str = 'Initial Position X=%s and Y=%s' %(X,Y)
    rospy.loginfo(str)
    pub.publish(str)

    # Create a SMACH state machine - with outcome 'finish'
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        # generic state

################################################################################
########### STATE MACHINE ######################################################
################################################################################
		#State Machine - This bit changes for each mission       
                
        smach.StateMachine.add('INITIALISE', Initialise(lib,15), #(lib, timeout)	This state ensure all systems are running correctly proir to starting the mission proper
            transitions={'succeeded':'ZIGZAG', 'aborted':'STOP','preempted':'STOP'})
        
        smach.StateMachine.add('ZIGZAG', zigzag(lib, 1100, 20, 100),
            transitions={'succeeded':'STOP', 'aborted':'STOP','preempted':'STOP'})

        smach.StateMachine.add('STOP', Stop(lib), 						  #This state stops the vehicle
            transitions={'succeeded':'finish'})    

################################################################################
########### END OF STATE MACHINE ###############################################
################################################################################

 

    # Create and start the introspection server - for visualisation
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    

    # Execute the state machine
    outcome = sm.execute()
    print 'finished executing state machine'

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

############################### SHUTDOWN FUNCTION ##############################

def shutdown(args=None):
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    
    #for debugging
    print 'entered shutdown'
    
    lib = library_highlevel()
    
    lib.stop()
            
################################################################################

if __name__ == '__main__':
    
    main()
    
    rospy.on_shutdown(shutdown)

