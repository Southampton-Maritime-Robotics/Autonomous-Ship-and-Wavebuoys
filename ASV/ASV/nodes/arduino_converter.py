#!/usr/bin/python
##############################################################################
#arduino_converter.py
#
#Code created by Enrico Anderlini (ea3g09@soton.ac.uk) for the conversion of the
#values published from and subscribed to the arduino. In a few words, this code
#changes the names of the publishers and subscribers that the arduino employs.
#This is necessary due to the very small RAM of the arduino (2 kB), which means
#that the publishers and subscribers names must be very short so as to employ 
#as few bytes as possible.
#
#Modifications to code
# 15/02/2013 Code created
#
##############################################################################
#Notes
#
#
#
##############################################################################

import roslib; roslib.load_manifest('ASV')
import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from ASV.msg import status

## Defining variables required in the subscribers and in the callback functions

global rudderdemand
global rudderD
global s1
global S1
global s2
global S2
global s3
global S3
global s4
global S4
global s5
global S5
global s6
global S6
global s7
global S7
global s8
global S8
global s9
global S9
global s10
global S10
global s11
global S11
global s12
global S12
global s13
global S13
global s14
global S14
global setmotorsetting
global MotorS 
global setmotortarget
global MotorT

## Initialise parameters

rudderdemand  = 0
s1            = 0
s2            = 0
s3            = 0
s4            = 0
s5            = 0
s6            = 0
s7            = 0
s8            = 0
s9            = 0
s10           = 0
s11           = 0
s12           = 0
s13           = 0
s14           = 0
s15           = 0
s16           = 0
setmotorsetting = 0
setmotortarget  = 0

def arduino_converter():
        rospy.init_node('arduino_converter')
        
        #Publish compass status as true!
        pubStatus = rospy.Publisher('status', status)
        
        ######################## Define Publishers #############################
        p1 = rospy.Publisher('s1', Float32)
        p2 = rospy.Publisher('s2', Int8)
        p3 = rospy.Publisher('s3', Float32)
        
        error           = rospy.Publisher('arduino_status', Int8)  #0 running OK, 1 error
        warning         = rospy.Publisher('arduino_warning', Int8)
        batteryvoltage  = rospy.Publisher('battery_voltage', Float32)
        casetemperature = rospy.Publisher('CaseTemperature', Float32)
        ruddertarget    = rospy.Publisher('rudder_target_angle', Float32)
        rudderangle     = rospy.Publisher('rudder_angle', Float32)
        motorsetting    = rospy.Publisher('motor_target_method', Int8)
        motortarget     = rospy.Publisher('motor_target', Float32)
        dutycycle       = rospy.Publisher('MotorDutyCycle', Float32)
        proprpm         = rospy.Publisher('prop_rpm', Float32)
        motorvoltage    = rospy.Publisher('motor_voltage', Float32)
        motorcurrent    = rospy.Publisher('motor_current', Float32)
        motorpower      = rospy.Publisher('motor_power', Float32)
        thrust          = rospy.Publisher('thrust', Float32)
        water_detected  = rospy.Publisher('water_detected', Int8)
        bilge_pump_on   = rospy.Publisher('bilge_pump', Int8)
        
        ####################### Define Subscribers #############################
        rospy.Subscriber('p1', Int8, s1_cb)
        rospy.Subscriber('p2', Int8, s2_cb) 
        rospy.Subscriber('p3', Float32, s3_cb)
        rospy.Subscriber('p4', Float32, s4_cb)
        rospy.Subscriber('p5', Float32, s5_cb)
        rospy.Subscriber('p6', Float32, s6_cb)
        rospy.Subscriber('p7', Int8, s7_cb)
        rospy.Subscriber('p8', Float32, s8_cb)
        rospy.Subscriber('p9', Float32, s9_cb)
        rospy.Subscriber('p10', Float32, s10_cb)
        rospy.Subscriber('p11', Float32, s11_cb)
        rospy.Subscriber('p12', Float32, s12_cb)
        rospy.Subscriber('p13', Float32, s13_cb)
        rospy.Subscriber('p14', Float32, s14_cb)
        rospy.Subscriber('p15', Int8, s15_cb)
        rospy.Subscriber('p16', Int8, s16_cb)
        
        rospy.Subscriber('rudder_demand', Float32, rudderd_cb)
        rospy.Subscriber('setMotorTargetMethod', Int8, setting_cb)
        rospy.Subscriber('setMotorTarget', Float32, target_cb)


        while not rospy.is_shutdown():
            
            NodeOn=True
            #Publish Dummy Values
            pubStatus.publish(nodeID = 2, status = NodeOn)
            
            ##Publishing the values read by the subscribers##
            p1.publish(rudderdemand)  
            p2.publish(setmotorsetting)
            p3.publish(setmotortarget)
            error.publish(s1)
            warning.publish(s2)
            batteryvoltage.publish(s3)        
            casetemperature.publish(s4)
            ruddertarget.publish(s5)
            rudderangle.publish(s6)
            motorsetting.publish(s7)
            motortarget.publish(s8)
            dutycycle.publish(s9)
            proprpm.publish(s10)
            motorvoltage.publish(s11)
            motorcurrent.publish(s12)
            motorpower.publish(s13)
            thrust.publish(s14)
            water_detected.publish(s15)
            bilge_pump_on.publish(s16)
            
            #### Publishing the values every 0.10 seconds - may need changing
            rospy.sleep(0.10)


################# Setting up callback functions for subscribers ###############

def rudderd_cb (rudderD):
    global rudderdemand
    rudderdemand = rudderD.data

def setting_cb (MotorS):
    global setmotorsetting
    setmotorsetting = MotorS.data

def target_cb (MotorT):
    global setmotortarget
    setmotortarget = MotorT.data
    
def s1_cb (S1):
    global s1
    s1 = S1.data
    
def s2_cb (S2):
    global s2
    s2 = S2.data
    
def s3_cb (S3):
    global s3
    s3 = S3.data
    
def s4_cb (S4):
    global s4
    s4 = S4.data
    
def s5_cb (S5):
    global s5
    s5 = S5.data
    
def s6_cb (S6):
    global s6
    s6 = S6.data
    
def s7_cb (S7):
    global s7
    s7 = S7.data
    
def s8_cb (S8):
    global s8
    s8 = S8.data
    
def s9_cb (S9):
    global s9
    s9 = S9.data
    
def s10_cb (S10):
    global s10
    s10 = S10.data
    
def s11_cb (S11):
    global s11
    s11 = S11.data
    
def s12_cb (S12):
    global s12
    s12 = S12.data
    
def s13_cb (S13):
    global s13
    s13 = S13.data
    
def s14_cb (S14):
    global s14
    s14 = S14.data
    
def s15_cb (S15):
    global s15
    s15 = S15.data
    
def s16_cb (S16):
    global s16
    s16 = S16.data

####################### Main Loop ########################  
if __name__ == '__main__':
	try:
            arduino_converter()
	except rospy.ROSInterruptException: pass
