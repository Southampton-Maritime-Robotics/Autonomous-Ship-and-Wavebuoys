#!/usr/bin/python
##############################################################################
#logger.py
#
#Initial code is a simplified subset of logger.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#
#Revisited and substantially modified by Enrico Anderlini (ea3g09@soton.ac.uk).
#
#Modifications to code
# 21/11/2012	Modified code for ASV	ABP
# 03/20/2013    Subscribers modified according to the other nodes.
#               Modified functions headingdemand_callback, rudderangledemand_callback,
#               propdemand_callback.
#               Created functions rudderangle_callback, proprpm_callback, 
#               voltage_callback and relative subscribers.
#               Deleted some useless lines (global variables).
#               Added more user friendly comments after checking the code works.
# 13/02/2013    Changed names of subscribers and added more subscribers in order
#               to read the data from the arduinos.
#
##############################################################################
#Notes
#
#This code writes a series of log files to /home/gdp40/logFiles (or ~/logFiles).
#This file will need to be modified to ensure that all your key variables are being logged.
#As of 03/02/2013, this file logs the values of the output from the compass (heading etc.)
#and that from the gps (position, velocity etc.), the heading demand, the rudder 
#angle demand, the actual rudder angle, 
#the battery voltage and the mission status. This can be seen from the employed
#subscribers at the end of the code. Note that these variable names must NOT be 
#changed as they are the same in all the nodes of the package ASV.
#
#In addition, various particulars of the motor, including power, voltage,
#current and propeller rpm are also logged from the arduinos.
#
#This file is consituted by one function for the logging of each of the aforementioned
#values at the corresponding instant in time. See the code for greater clarity.
#
##############################################################################
import roslib; roslib.load_manifest('ASV')
import rospy
import csv
import time
import numpy
import os
from datetime import datetime
from ASV.msg import compass
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import String
from ASV.msg import position


global compassWriter
global compassFile
global heading_demand
global rudder_demand
global motor_setting
global motor_demand
global prop_rpm
global motor_voltage
global motor_current
global motor_power
global temperature
global dutycycle
global thrust
global positionFile
global positionWriter
global time_zero
global pathFile
global log_folder

###############################################################
#The following functions write the values this node subscribes to into different
#log files in .cvs format within the folder ~/logFiles created within the main
#function.
###############################################################

#This function outputs compass data to file compassLog.csv: time, heading, roll, pitch, temperature,  m, mx, my, mz, a, ax, ay, az
def callback_compass(compass_data):
    #The stringtime variable is used in all these functions to store the time of
    #the reading (starting from the time of the start-up (zero))-expressed in seconds.
    stringtime = time.time()-time_zero
    compassList = [stringtime, compass_data.heading, compass_data.roll, compass_data.pitch, compass_data.temperature,  compass_data.m, compass_data.mx, compass_data.my, compass_data.mz, compass_data.a, compass_data.ax, compass_data.ay, compass_data.az]

    with open('%s/compassLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(compassList)
        except ValueError:
            print 'writerow error'

##############################################################

#This function stores the output of the states to file heading_demandLog.csv: time, heading angle demand (in degrees)
def headingdemand_callback(heading_demand):    
    stringtime = time.time()-time_zero
    headingDList = [stringtime, heading_demand.data]            #adding the .data results in the node reading the floating number only
    #The following command creates (opens) the .cvs file in order to write the 
    #values to it.
    with open('%s/heading_demandLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(headingDList)
        except ValueError:
            print 'writerow error'

#################################################################

#This function logs the output of the heading controller to file Setrudder_demandLog.csv: time, rudder angle demand (degrees).
def rudderangledemand_callback(rudder_demand):    
    stringtime = time.time()-time_zero
    rudderDList = [stringtime, rudder_demand.data]             #adding the .data results in the node reading the floating number only
    with open('%s/Setrudder_demandLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(rudderDList)
        except ValueError:
            print 'writerow error'
            
###################################################################    

#This function logs the output of the heading controller to file rudder_demandLog.csv: time, rudder angle demand (degrees) 
#as read by the arduino- crosscheck.
def targetangle_callback(rudder_demand):    
    stringtime = time.time()-time_zero
    rudderDList = [stringtime, rudder_demand.data]             #adding the .data results in the node reading the floating number only
    with open('%s/rudder_demandLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(rudderDList)
        except ValueError:
            print 'writerow error'
            
################################################################### 

#This function outputs the arduino data to file rudder_angleLog.csv: time, rudder angle (degrees).
def rudderangle_callback(rudder_angle):     
    stringtime = time.time()-time_zero
    rudderAList = [stringtime, rudder_angle.data]             #adding the .data results in the node reading the floating number only
    with open('%s/rudder_angleLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(rudderAList)
        except ValueError:
            print 'writerow error'

###################################################################

#This function logs the output of the states to file Setmotor_settingLog.csv: time, motor setting:  1:voltage, 2:rpm, 3:power.
def setmotorsetting_callback(motor_setting):
    stringtime = time.time()-time_zero
    motorsettingList = [stringtime, motor_setting.data]          #adding the .data results in the node reading the floating number only
    with open('%s/Setmotor_settingLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(motorsettingList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function logs the output of the states to file motor_settingLog.csv: time, motor setting:  1:voltage, 2:rpm, 3:power.
#as seen by the arduino - crosscheck
def motorsetting_callback(motor_setting):
    stringtime = time.time()-time_zero
    motorsettingList = [stringtime, motor_setting.data]          #adding the .data results in the node reading the floating number only
    with open('%s/motor_settingLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(motorsettingList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function logs the output of the states to file Setmotor_demandLog.csv: time, motor demand (rpm, V, or W).
def setmotordemand_callback(motor_demand):
    stringtime = time.time()-time_zero
    motordemandList = [stringtime, motor_demand.data]          #adding the .data results in the node reading the floating number only
    with open('%s/Setmotor_demandLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(motordemandList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function logs the output of the states to file motor_demandLog.csv: time, motor demand (rpm, V, or W).
#as seen by the arduino - crosscheck
def motordemand_callback(motor_demand):
    stringtime = time.time()-time_zero
    motordemandList = [stringtime, motor_demand.data]          #adding the .data results in the node reading the floating number only
    with open('%s/motor_demandLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(motordemandList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function outputs the arduino data to file prop_rpmLog.csv: time, propeller rpm.
def proprpm_callback(prop_rpm):
    stringtime = time.time()-time_zero
    proprpmList = [stringtime, prop_rpm.data]             #adding the .data results in the node reading the floating number only
    with open('%s/prop_rpmLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(proprpmList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function outputs the arduino data to file motor_voltageLog.csv: time, motor voltage.
def motorvoltage_callback(motor_voltage):
    stringtime = time.time()-time_zero
    motorvoltageList = [stringtime, motor_voltage.data]             #adding the .data results in the node reading the floating number only
    with open('%s/motor_voltageLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(motorvoltageList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function outputs the arduino data to file motor_currentLog.csv: time, motor current.
def motorcurrent_callback(motor_current):
    stringtime = time.time()-time_zero
    motorcurrentList = [stringtime, motor_current.data]             #adding the .data results in the node reading the floating number only
    with open('%s/motor_currentLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(motorcurrentList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function outputs the arduino data to file motor_powerLog.csv: time, motor power.
def motorpower_callback(motor_power):
    stringtime = time.time()-time_zero
    motorpowerList = [stringtime, motor_power.data]             #adding the .data results in the node reading the floating number only
    with open('%s/motor_powerLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(motorpowerList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function outputs the arduino data to file thrustLog.csv: time, thrust.
def thrust_callback(thrust):
    stringtime = time.time()-time_zero
    thrustList = [stringtime, thrust.data]             #adding the .data results in the node reading the floating number only
    with open('%s/thrustLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(thrustList)
        except ValueError:
            print 'writerow error'
    
##############################################################

#This function outputs gps data to file positionLog.csv: time, x-position, y-position, speed, latitude, longitude, GPS flag 
def position_callback(position):
    
    stringtime = time.time()-time_zero
    positionList = [stringtime, position.X, position.Y, position.speed, position.lat, position.long, position.ValidGPSfix]
    
    with open('%s/positionLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(positionList)
        except ValueError:
            print 'writerow error'
##############################################################

#This function outputs the arduino data to file voltageLog.csv: time, battery voltage
def voltage_callback(battery_voltage):
    
    stringtime = time.time()-time_zero
    voltageList = [stringtime, battery_voltage.data]   #adding the .data results in the node reading the floating number only
    
    with open('%s/voltageLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(voltageList)
        except ValueError:
            print 'writerow error'
            
################################################################

#This function outputs the arduino data to file temperatureLog.csv: time, temperature inside the box
def temperature_callback(temperature):
    
    stringtime = time.time()-time_zero
    temperatureList = [stringtime, temperature.data]   #adding the .data results in the node reading the floating number only
    
    with open('%s/temperatureLog.csv' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(temperatureList)
        except ValueError:
            print 'writerow error'
            
################################################################

#This function outputs the arduino data to file dutycycleLog.csv: time, motor duty cycle
#def dutycycle_callback():
    
#    stringtime = time.time()-time_zero
#    dutycycleList = [stringtime, .data]   #adding the .data results in the node reading the floating number only
    
#    with open('%s/dutycycleLog.csv' %(dirname), "a") as f:
#        try:
#            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#            Writer.writerow(dutycycleList)
#        except ValueError:
#            print 'writerow error'
            
################################################################

#This function outputs the mission status to the file mission.txt (only strings)
def mission_callback(HC):                 #I assume HC stands for heading controller, but this is not the case in ASV-to be modified
    stringtime = time.time()-time_zero
    missionList = [stringtime, HC.data]
    
    with open('%s/mission.txt' %(dirname), "a") as f:
        try:
            Writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            Writer.writerow(missionList)
        except ValueError:
            print 'writerow error'

##############################################################

def shutdown():
    #shutdown behaviour - close all files
    #print 'shutting down'
    with open('%s/path.kml' %(dirname), "a") as f:
        try:  
            f.write('</coordinates>\n </LineString>\n </Placemark>\n </kml>\n')
        except ValueError:
            print 'write error'

################################################################################
####### INITIALISE #############################################################
################################################################################

if __name__ == '__main__':
    rospy.init_node('Logger')
    
    stringtime = datetime.now()
    stringtime = stringtime.strftime('%Y-%m-%d_%H-%M-%S')
    rospy.loginfo('Logger started at %s.'%(stringtime))
    pub_folder = rospy.Publisher('folder', String)
    global time_zero

    time_zero = time.time()

  
################################################################################
######## FOLDERS ###############################################################
################################################################################
  
    #define files and writers
    logfolder =  'logFiles'   
    dirname   = logfolder + '/' + stringtime
    
    if not os.path.isdir(logfolder):
        print 'made logfolder'
        os.mkdir(logfolder)
    if not os.path.isdir(dirname):
        print 'made test folder'
        os.mkdir(dirname)
        
    time.sleep(5)
    pub_folder.publish(dirname)


################################################################################
######## SUBSCRIBERS ###########################################################
################################################################################

    rospy.Subscriber('compass_out', compass, callback_compass)     
    rospy.Subscriber('heading_demand', Float32, headingdemand_callback)      
    rospy.Subscriber('rudder_demand', Float32, rudderangledemand_callback)
    rospy.Subscriber('rudder_target_angle', Float32, targetangle_callback)
    rospy.Subscriber('rudder_angle', Float32, rudderangle_callback)
    rospy.Subscriber('setMotorTargetMethod', Int8, setmotorsetting_callback) #1 voltage, 2 rpm, 3 power
    rospy.Subscriber('motor_target_method', Int8, motorsetting_callback) #1 voltage, 2 rpm, 3 power
    rospy.Subscriber('setMotorTarget', Float32, setmotordemand_callback)
    rospy.Subscriber('motor_target', Float32, motordemand_callback)
    rospy.Subscriber('prop_rpm', Float32, proprpm_callback)
    rospy.Subscriber('motor_voltage', Float32, motorvoltage_callback)
    rospy.Subscriber('motor_current', Float32, motorcurrent_callback)
    rospy.Subscriber('motor_power', Float32, motorpower_callback)
    rospy.Subscriber('thrust', Float32, thrust_callback)
    rospy.Subscriber('position',position, position_callback)
    rospy.Subscriber('battery_voltage', Float32, voltage_callback)
    rospy.Subscriber('CaseTemperature', Float32, temperature_callback)
#    rospy.Subscriber('MotorDutyCycle', Float32, dutycycle_callback)
    rospy.Subscriber('MissionStrings', String, mission_callback)
    #rospy is the ROS python packages that enables the creation of subscribers
    #and publishers. The name of the subscribers between '' must NOT be changed
    #as they are the same in all nodes within the ASV package. Changing them 
    #only in one node will result in that node not being able to perform its 
    #function (publishing/subscribing).


    str = "Logger online - output directory: %s" %(dirname)
    rospy.loginfo(str)
    
    rospy.on_shutdown(shutdown)

    rospy.spin()
    
