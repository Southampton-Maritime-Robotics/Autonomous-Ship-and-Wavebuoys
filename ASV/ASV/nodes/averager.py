#!/usr/bin/python
##############################################################################
#averager.py
#
#This code has been created by Enrico Anderlini (ea3g09@soton.ac.uk) for 
#averaging the main readings required during the QinetiQ tests. These values
#averaged over one minute will be published to an external logfile.
#
#Modifications to code
#16/02/2013     code created
#17/02/2013     removal of the calls to library_highlevel.py because whenever
#               one of the nodes was not being published the node exited with
#               errors.
#
##############################################################################
#Notes
#
#At the moment this file publishes to an external log file the values for the
#motor demand (rpm, voltage or power), the propeller rpm, the motor voltage or
#power, the battery voltage and the case temperature (hence, 4 values in total
#plus the time at which they have been sampled). Other variables may be added
#as required. 
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import time
import csv
import os
import numpy
from datetime import datetime
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import String
from ASV.msg import status

# Defining global variables
global time_zero
global counter
global Motor_setting
global Motor_target
global total_motor
global Prop_rpm
global total_rpm
global avg_rpm
global Voltage
global total_voltage
global avg_voltage
global Motor_current
global total_current
global avg_current
global Power
global total_power
global avg_power
global battery_voltage
global total_BatteryVoltage
global avg_BatteryVoltage
global Temperature
global total_temperature
global avg_temperature
global Thrust
global total_thrust
global avg_thrust

###############################################################
#The following functions write the values this node subscribes to into different
#log files in .cvs format within the folder ~/logFiles created within the main
#function.
############################################################### 

def printer(setting, target, rpm, voltage, current, power, BatteryVoltage, temperature, thrust):
    #The stringtime variable is used in all these functions to store the time of
    #the reading (starting from the time of the start-up (zero))-expressed in seconds.
    stringtime = time.time()-time_zero
    averageList = [stringtime, setting, target, rpm, voltage, current, power, BatteryVoltage, temperature, thrust]
    title = ['time', 'setting', 'target', 'rpm', 'volt', 'current', 'power', 'battery', 'temp', 'thrust']

    print title
    print averageList
    
    with open('%s/averageLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(title)
            Writer.writerow(averageList)
        except ValueError:
            print 'writerow error'

########################## Callback Functions #################################

def motor_setting_cb(Motor_setting):
    global motor_setting
    motor_setting = Motor_setting.data

def motor_target_cb(Motor_target):
    global motor_target
    motor_target = Motor_target.data
    
def prop_rpm_cb(Prop_rpm):
    global prop_rpm
    prop_rpm = Prop_rpm.data
    
def motor_voltage_cb(Voltage):
    global voltage
    voltage = Voltage.data
    
def motor_current_cb(Motor_current):
    global motor_current
    motor_current = Motor_current.data
    
def motor_power_cb(Motor_power):
    global motor_power
    motor_power = Motor_power.data

def thrust_cb(Thrust):
    global thrust
    thrust = Thrust.data

def battery_voltage_cb(battery_voltage):
    global BatteryVoltage
    BatteryVoltage = battery_voltage.data
    
def temperature_cb(Temperature):
    global temperature
    temperature = Temperature.data
            
##############################################################

#def shutdown():
    #shutdown behaviour - close all files
    #print 'shutting down'
#    with open('%s/path.kml' %(dirname), "a") as f:
#        try:  
#            f.write('</coordinates>\n </LineString>\n </Placemark>\n </kml>\n')
#        except ValueError:
#            print 'write error'
            
################################## MAIN FUNCTION ###############################

if __name__ == '__main__':
        
            #Initialising the node
        rospy.init_node('averager')
        
        stringtime = datetime.now()
        stringtime = stringtime.strftime('%Y-%m-%d_%H-%M-%S')
        rospy.loginfo('Logger started at %s.'%(stringtime))
        pub_folder = rospy.Publisher('folder', String)
        
        ########################################################################
        ######## FOLDERS #######################################################
        ########################################################################
  
        #define files and writers
        logfolder =  'AverageValues'   
        dirname   = logfolder + '/' + stringtime
        
        if not os.path.isdir(logfolder):
            print 'made logfolder'
            os.mkdir(logfolder)
        if not os.path.isdir(dirname):
            print 'made test folder'
            os.mkdir(dirname)
        
        time.sleep(5)
        pub_folder.publish(dirname)

        ########################################################################

        #Setting the zero time
        time_zero = time.time()

        # Initialising global variables
        counter         =0
        motor_setting   =0
        motor_target    =0
        prop_rpm        =0
        voltage         =0
        motor_current   =0
        motor_power     =0
        BatteryVoltage  =0
        temperature     =0
        thrust          =0
        total_motor     =0
        avg_motor       =0
        total_rpm       =0
        avg_rpm         =0
        total_voltage   =0
        avg_voltage     =0
        total_current   =0
        avg_current     =0
        total_power     =0
        avg_power       =0
        total_BatteryVoltage    =0
        avg_BatteryVoltage      =0
        total_temperature       =0
        avg_temperature         =0
        total_thrust            =0
        avg_thrust              =0

        ########################SET UP THE SUBSCRIBERS##########################
        rospy.Subscriber('setMotorTargetMethod', Int8, motor_setting_cb)
        rospy.Subscriber('setMotorTarget', Float32, motor_target_cb)
        rospy.Subscriber('prop_rpm', Float32, prop_rpm_cb)
        rospy.Subscriber('motor_voltage', Float32, motor_voltage_cb)
        rospy.Subscriber('motor_current', Float32, motor_current_cb)
        rospy.Subscriber('motor_power', Float32, motor_power_cb)
        rospy.Subscriber('thrust', Float32, thrust_cb)
        rospy.Subscriber('battery_voltage', Float32, battery_voltage_cb)
        rospy.Subscriber('CaseTemperature', Float32, temperature_cb)
        
        
        #Publish the propeller rpm demand only when the node is not shutdown
        #while not rospy.is_shutdown():
            
        while (time.time()-time_zero)<=20:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
        
        while (time.time()-time_zero)>20 and (time.time()-time_zero)<=40:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
        
        while (time.time()-time_zero)>40 and (time.time()-time_zero)<=60:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
            
        while (time.time()-time_zero)>60 and (time.time()-time_zero)<=80:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
            
        while (time.time()-time_zero)>80 and (time.time()-time_zero)<=100:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
            
        while (time.time()-time_zero)>100 and (time.time()-time_zero)<=120:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
            
        while (time.time()-time_zero)>120 and (time.time()-time_zero)<=140:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
        
        while (time.time()-time_zero)>140 and (time.time()-time_zero)<=160:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
        
        
        while (time.time()-time_zero)>160 and (time.time()-time_zero)<=180:  
              
            counter       = counter + 1
            total_rpm     = prop_rpm  + total_rpm
            total_voltage = voltage + total_voltage
            total_current = motor_current + total_current
            total_power   = motor_power + total_power
            total_BatteryVoltage = BatteryVoltage
            total_temperature    = temperature + total_temperature
            total_thrust         = thrust + total_thrust
            rospy.sleep(0.1) 
            
            #For debugging purposes only
            #print counter
            
        avg_rpm     = total_rpm / (counter+1)
        avg_voltage = total_voltage / (counter+1)
        avg_current = total_current / (counter+1)
        avg_power   = total_power / (counter+1)
        avg_BatteryVoltage = total_BatteryVoltage / (counter+1)
        avg_temperature    = total_temperature / (counter+1)
        avg_thrust         = total_thrust / (counter+1)

        
        printer(motor_setting, motor_target, avg_rpm, avg_voltage, avg_current, avg_power, avg_BatteryVoltage, avg_temperature, avg_thrust)
        
