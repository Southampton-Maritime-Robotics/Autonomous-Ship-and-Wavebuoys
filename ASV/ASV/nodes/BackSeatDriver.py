#!/usr/bin/env python
##############################################################################
#BackSeatDriver.py
#
#Initial code is a simplified subset of BackSeatDriver.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#Code modified by Enrico Anderlini (ea3g09@soton.ac.uk) for adaption to 
#modifications in the electronics hardware.
#
#
#Modifications to code
# 21/11/2012	Modified code for ASV	ABP
# 16/02/2013    Addition of checks for arduino status and temperature within 
#               the arduino electronics box
# 08/03/2013    Addition of checks for arduino warnings, water detection and
#               bilge pump status.
#
##############################################################################
#Notes
#
#This node is the backseat driver: it sits in the background and polls the status of the vehicle.
#In the event of an issue it publishes a 1 to back_seat_flag which will trigger an interupt in the mission controller.
#If an event is deemed to require an immediate stop this should be triggered in this code.
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import roslib
import rospy
import time
from std_msgs.msg import Int8
from std_msgs.msg import String
from library_highlevel import library_highlevel

def main(controller):
    rospy.init_node('back_seat_driver')
    pub=rospy.Publisher('back_seat_flag',Int8)
    pub2 = rospy.Publisher('MissionStrings', String)
    
    # Store Initial Time
    time_zero = time.time()    
    
    time.sleep(5) #Allow critical systems to come online.

  
    # Import Limit Parameters from launch file
    overPitch = rospy.get_param('over-pitch')    
    overRoll = rospy.get_param('over-roll')     
    maxInternalTemp = rospy.get_param('max-internal-temp')  
    minBatteryVoltage = rospy.get_param('min-battery-voltage')
    missionTimeout = rospy.get_param('mission-timeout')
    missionTimeout = missionTimeout*60          #Mission timeout in minutes therefore convert to seconds
    
    
    #Initialise BackSeatFlag to zero
    BackSeatFlag=0
    pub2.publish('Backseat Driver Node Is Active')
    
    while not rospy.is_shutdown():
        time.sleep(0.01)
        time_elapsed=time.time()-time_zero
    
    
        #Poll System For Any Potential Errors or Status Warnings
    
       
        #Identify OverPitch?
        current_pitch=controller.getPitch()
        
        if abs(current_pitch) > overPitch: 
            BackSeatFlag=1
            print "Excessive pitch!"
            str = "Current pitch %sdeg > Pitch limit of %sdeg" %(current_pitch, overPitch) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return
        
        #Identify OverRoll?
        current_roll=controller.getRoll()
        
        if abs(current_roll) > overRoll: 
            BackSeatFlag=1
            str = "Current roll %sdeg > Roll limit of %sdeg" %(current_roll, overRoll) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return        

        #Check Internal Pressure Vessel Temperature
        current_temperature=controller.getTemperature()
        
        if current_temperature>maxInternalTemp: 
            BackSeatFlag=1
            print "Temperature in Pelicase is too high!"
            str = "Current temperature %sdeg > Temperature limit of %sdeg" %(current_temperature, maxInternalTemp) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return   
        
        #Check Temperature inside electronics box
        electronics_temperature=controller.getCaseTemperature()
        
        if electronics_temperature>maxInternalTemp: 
            BackSeatFlag=1
            print "Temperature in Electronics box is too high!"
            str = "Current temperature %sdeg > Temperature limit of %sdeg" %(electronics_temperature, maxInternalTemp) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return
        
        #Check arduino status for errors -- characterized by 1
        arduino_status=controller.getArduinoStatus()
        
        if arduino_status==1: 
            BackSeatFlag=1
            print "Arduino Error! - Lost contact"
            str = "Arduino error" 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return
        
        #Check arduino status for errors -- characterized by 1
        arduino_warning=controller.getArduinoWarning()
        
        if arduino_warning==1: 
            BackSeatFlag=0
            print "Arduino warning"
            str = "Arduino warning" 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return
        
        #Check whether water has been detected (0 no, 1 yes)
        water_detected=controller.getWaterStatus()
        
        if water_detected==1: 
            BackSeatFlag=0            #It may be subjected to changes in the future
            print "We're shipping water!"
            str = "Water Detected" 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return
        
        #Check for bigle pump status
        bilge_pump=controller.getBilgePumpStatus()
        
        if bilge_pump==1: 
            BackSeatFlag=0
            print "Bilge pump working"
            str = "Bilge pump on" 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return
        
        #Check battery Voltage
        current_voltage=controller.getBatteryVoltage()
        
        if current_voltage<minBatteryVoltage: 
            BackSeatFlag=1
            print "I'm running out of fuel!"
            str = "Current voltage %smV < Battery voltage limit of %smV" %(current_voltage, minBatteryVoltage) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return                 
        
        #Check Mission Duration
        current_time=time.time()-time_zero
        
        if current_time>missionTimeout: 
            BackSeatFlag=1
            print "Mission timeout"
            str = "Current mission time %ss > Mission time limit of %ss" %(current_time, missionTimeout) 
            rospy.logerr(str)
            pub.publish(BackSeatFlag)
            pub2.publish(str)
            return   
        
if __name__ == '__main__':
    # Define an instance of highlevelcontrollibrary to pass to all action servers
    lib = library_highlevel()
    main(lib)
    rospy.spin()
