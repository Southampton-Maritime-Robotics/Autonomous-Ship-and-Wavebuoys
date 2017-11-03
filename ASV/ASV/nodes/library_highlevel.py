#!/usr/bin/python
##############################################################################
#library_highlevel.py
#
#Initial code is a simplified subset of library_highlevel.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#Revisited and corrected by Enrico Anderlini (ea3g09@soton.ac.uk).
#
#Modifications to code
# 21/11/2012	Modified code for ASV	ABP
# 03/02/2013    Corrected subscribers/publishers names in accordance with their 
#               variables in the other nodes
# 13/02/2013    Changing subscribers and publishers names and adding some
#               subscribers in order to work with the arduinos
# 14/02/2013    Modifying setter and getter functions
# 16/02/2013    Finalisation and test of the node
#
##############################################################################
#Notes
#
#This code contains the background coding required to provide get and set functions for
#the other high level code to use
#
#The arduino status may create problems 
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import numpy
import math
import time
import re

#import message types for publishing:
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String

#import message types for subscribing:
from ASV.msg import compass
from ASV.msg import position
from ASV.msg import status



class library_highlevel:
# library class for high-level controller.  Defines the inputs, output and basic functionality that
# all high-level controllers should inherit.
	

    def __init__(self):
        #constructor method - creates objects of type 'HighLevelController'
        
        #set up publishers (stored as object variables)
        self.pub_motor_target              = rospy.Publisher('setMotorTarget',Float32)    #sets motor target
        self.pub_motor_setting             = rospy.Publisher('setMotorTargetMethod',Int8) #selects type of demand
        self.pub_heading_demand            = rospy.Publisher('heading_demand', Float32) 
        self.Mission_pub                   = rospy.Publisher('MissionStrings', String)
        self.pub_heading_control_onOff     = rospy.Publisher('Heading_onOFF', Bool)             #turns heading controller on and off
        self.pub_rudder_demand   	   = rospy.Publisher('rudder_demand', Float32)
	self.pub_manoeuvring		   = rospy.Publisher('manoeuvring', Int8)


        
        #set up subscribers
        rospy.Subscriber('compass_out', compass, self.callback_compass)                         
        rospy.Subscriber('position', position, self.callback_position)                     		
        rospy.Subscriber('back_seat_flag', Int8, self.callback_back_seat_flag)
        rospy.Subscriber('status', status, self.callback_status)
        rospy.Subscriber('rudder_target_angle', Float32, self.callback_target_angle)  
        rospy.Subscriber('rudder_angle', Float32, self.callback_rudder_angle)     
        rospy.Subscriber('prop_rpm', Float32, self.callback_prop_rpm)
        rospy.Subscriber('motor_voltage', Float32, self.callback_motor_voltage)  
        rospy.Subscriber('motor_current', Float32, self.callback_motor_current)  
        rospy.Subscriber('motor_power', Float32, self.callback_motor_power)  
        rospy.Subscriber('battery_voltage', Float32, self.callback_battery_voltage)
        rospy.Subscriber('motor_target_method', Int8, self.callback_motor_method) 
        rospy.Subscriber('motor_target', Float32, self.callback_motor_target) 
        rospy.Subscriber('MotorDutyCycle', Float32, self.callback_motor_dutycycle) 
        rospy.Subscriber('CaseTemperature', Float32, self.callback_case_temperature)
        rospy.Subscriber('arduino_status', Int8, self.callback_arduino_status)
        rospy.Subscriber('arduino_warning', Int8, self.callback_arduino_warning)  
        rospy.Subscriber('water_detected', Int8, self.callback_water_detected)
        rospy.Subscriber('bilge_pump', Int8, self.callback_bilge_pump)
              
	    
        #initilise empty object variables to hold latest data from subscribers
        ######### might need to set default values in message files-not done
        self.__compass = compass()
        self.__position = position()

        self.__back_seat_flag  = 0 #back_seat_flag is 0 when no errors are present
        self.__battery_voltage = 0      
        self.__gps_status      = 0
        self.__compass_status  = 0
        self.__arduino_status  = 0
        self.__water_detected  = 0
        self.__bilge_pump      = 0
        self.__rudder_angle    = 0.0
        self.__prop_rpm        = 0.0   
        self.__motor_voltage   = 0
        self.__motor_current   = 0
        self.__motor_power     = 0
        self.__motor_method    = 0
        self.__motor_target    = 0
        self.__motor_dutycycle = 0
        self.__case_temperature= 0
        

	#Below are the various functions that can be called by the mission control node
	###############################################################################



    # stops vehicle and shuts down ROS
    def stop(self):
        str="Stop method invoked - ROS will shut down in 1 second"
        rospy.logfatal(str)
        self.setMotorTarget(0)
        self.setMotorSetting(0)
        self.setRudderAngle(0)
        time.sleep(1)
        rospy.logfatal("Shutting down")
        self.Mission_pub.publish('Shutting Down')
        rospy.signal_shutdown('mission finished')
        
    ##The following are the "setter" functions->publishers##
    
    # sets the motor operating mode: 1 for duty cycle, 2 for motor voltage,
    #3 for propeller rpm, 4 for motor power 
    def setMotorSetting(self, demand):
        str = "Setting motor operating mode %s" %demand			
        rospy.loginfo(str)
        self.pub_motor_setting.publish(demand)
        
    # sets motor target, in general
    
    def setMotorTarget(self,demand):
        str = "Setting motor target %s" %demand			
        rospy.loginfo(str)
        self.pub_motor_target.publish(demand)
    
    # sets a 'demand' for the rear prop 
    def setProp(self, demand):
        str = "Setting rear prop demand %s" %demand			
        rospy.loginfo(str)
        self.setMotorSetting(3)             #see arduino
        self.pub_motor_target.publish(demand)
        
    # sets a 'demand' for the motor voltage (percentage)
    def setMotorVoltage(self, demand):
        str = "Setting motor voltage %s" %demand			
        rospy.loginfo(str)
        self.setMotorSetting(2)             #see arduino
        self.pub_motor_target.publish(demand)
    
    # sets a 'demand' for the motor power (percentage)
    def setMotorPower(self, demand):
        str = "Setting motor power %s" %demand			
        rospy.loginfo(str)
        self.setMotorSetting(4)             #see arduino
        self.pub_motor_target.publish(demand)
    
    #sets a 'demand' for the motor duty cycle (0-255)
    def setDutyCycle(self, demand):
        str = "Setting motor duty cycle %s" %demand			
        rospy.loginfo(str)
        self.setMotorSetting(1)             #see arduino
        self.pub_motor_target.publish(demand)
                
    # set a 'demand' (in degrees) for the rudder angle
    def setRudderAngle(self, demand):
        str = "Ruddder demand %.3f deg" %demand
        rospy.loginfo(str)
        #publish rear rudder_demand
        self.switchHeadingOnOff
        self.pub_rudder_demand.publish(demand)


    # move to heading 'demand' (degrees)
    def setHeading(self, demand):
        #publish headingDemand
        cur_heading=self.getHeading()
        str = "Setting heading demand %.3f deg, current heading %.3f deg" %(demand, cur_heading)
        rospy.loginfo(str)
        self.switchHeadingOnOff(1)					#Turn on Heading control
        self.pub_heading_demand.publish(demand)
		
	# turn off heading controller)
    def setManoeuvring(self, demand):
        str = "Turning off heading controller momentarily" 
        rospy.loginfo(str)
        self.pub_manoeuvring.publish(demand)

    # change heading by 'headingChange' (degrees)
    def changeHeadingBy(self, headingChange):
        self.setHeading(self.__compass.heading + headingChange)


    # switch heading controller on or off {1,0}
    def switchHeadingOnOff(self,onOff):
        if onOff ==1:
            self.pub_heading_control_onOff.publish(1)
            str = "Switch Heading Control ON"
            rospy.logdebug(str)	 
        else:
            self.pub_heading_control_onOff.publish(0)
            str = "Switch Heading Control OFF"
            rospy.logdebug(str)	 

    #################################################
    ## "Getter" methods->subscribers##    
    
    def getHeading(self):
        return self.__compass.heading
    
    def getRoll(self):
        return self.__compass.roll
    
    def getPitch(self):
        return self.__compass.pitch
    
    def getTemperature(self):
        return self.__compass.temperature
    
   
    # magnetometer
    def getM(self):
        return self.__compass.m
    
    def getMx(self):
        return self.__compass.mx
    
    def getMy(self):
        return self.__compass.my
    
    def getMz(self):
        return self.__compass.mz
    
    # accelerometer
    def getA(self):
        return self.__compass.a
    
    def getAx(self):
        return self.__compass.ax
    
    def getAy(self):
        return self.__compass.ay
    
    def getAz(self):
        return self.__compass.az

    # get position values                         
    def getX(self):
        return self.__position.X

    def getY(self):
        return self.__position.Y
    
    def getGPSValidFix(self):
        return self.__position.ValidGPSfix    
                  
    def getVoltage(self):
        return self.__voltage
    
    def getBackSeatErrorFlag(self):
        return self.__back_seat_flag

	# get status
    def getGPSStatus(self):
        return self.__gps_status
    
    def getCompassStatus(self):
        return self.__compass_status

    def getArduinoStatus(self):
        return self.__arduino_status
    
    def getArduinoWarning(self):
        return self.__arduino_warning
    
    def getWaterStatus(self):
        return self.__water_detected
    
    def getBilgePumpStatus(self):
        return self.__bilge_pump
    
    # get rudder demand
    
    def getRudderDemand(self):
        return self.__TargetAngle

    # get rudder postion

    def getRudderAngle(self):
        return self.__RudderAngle
    
    # get motor demand
    
    def getMotorSetting(self):
        return self.__MotorMethod
    
    def getMotorTarget(self):
        return self.__MotorTarget
    
    # get motor particulars

    def getPropRPM(self):
        return self.__PropRPM
    
    def getMotorVoltage(self):
        return self.__MotorVoltage
    
    def getMotorCurrent(self):
        return self.__MotorCurrent
    
    def getMotorPower(self):
        return self.__MotorPower
    
    def getMotorMethod(self):
        return self.__MotorMethod

    # get battery voltage

    def getBatteryVoltage(self):
        return self.__BatteryVoltage
    
    # get case temperature
    
    def getCaseTemperature(self):
        return self.__CaseTemperature
    

    #################################################
    # Callbacks
    #################################################

    def callback_compass(self, compass_data):
           self.__compass = compass_data
    
    def callback_position(self, position):
           self.__position = position   
                   
    def callback_back_seat_flag(self, back_seat_flag):
           self.__back_seat_flag = back_seat_flag.data      
    
    # The following function can have problems with the arduino status
    
    def callback_status(self, status):
            if status.nodeID == 1:
                self.__arduino_status = status.status
                return
            elif status.nodeID == 4:
                self.__gps_status = status.status
                return
            elif status.nodeID == 5:
                self.__compass_status = status.status
                return
            
    #### The following readings are all taken from the arduinos ####

    def callback_target_angle(self, target_angle):
           self.__TargetAngle = target_angle.data
           
    def callback_rudder_angle(self, rudder_angle):
           self.__RudderAngle = rudder_angle.data
    
    def callback_prop_rpm(self, prop_rpm):
           self.__PropRRPM = prop_rpm.data   
                   
    def callback_battery_voltage(self, battery_voltage):
           self.__BatteryVoltage = battery_voltage.data  

    def callback_motor_voltage(self, motor_voltage):
           self.__MotorVoltage = motor_voltage.data 

    def callback_motor_current(self, motor_current):
           self.__MotorCurrent = motor_current.data 
           
    def callback_motor_power(self, motor_power):
           self.__MotorPower = motor_power.data 
           
    def callback_motor_method(self, motor_method):
           self.__MotorMethod = motor_method.data 
           
    def callback_motor_target(self, motor_target):
           self.__MotorTarget = motor_target.data 
           
    def callback_motor_dutycycle(self, motor_dutycycle):
           self.__MotorDutyCycle = motor_dutycycle.data 
           
    def callback_case_temperature(self, case_temperature):
           self.__CaseTemperature = case_temperature.data 
           
    def callback_arduino_status(self, ArduinoStatus):
           self.__arduino_status = ArduinoStatus.data 
           
    def callback_arduino_warning(self, ArduinoWarning):
           self.__arduino_warning = ArduinoWarning.data 
           
    def callback_water_detected(self, WaterDetected):
           self.__water_detected = WaterDetected.data
           
    def callback_bilge_pump(self, BilgePump):
           self.__bilge_pump = BilgePump.data
