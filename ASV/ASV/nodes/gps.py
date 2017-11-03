#!/usr/bin/python
#######################################################
#
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#Code modified and adapted for use in ASV package by Enrico Anderlini 
#(ea3g09@soton.ac.uk).
#
#Modifications to code
# 15/2/2012 converted speed to m/s
# 6/1/2012 modified calculation of X and Y variables
# 6/1/2012 modified code so position is always published
# 6/1/2012 lat/long origin now read from parameter server
##11/12/2012  adaption to use in the ASV BP.
##These modifications are highlighted with ##.
#12/12/2012 modifications required for new Python version (2.7)
#13/12/2012 modifications required because the library_highlevel.py uses the 
#file position.msg instead of gps.msg. Hence, also all states do indirectly use 
#this file. Hence, the code has been modified to work with position.msg.
#04/02/2013 extensive comments added
#05/02/2013 publisher "gps_out" substituted with "position" in accordance with 
#           the rest of the code
#05/03/2013 io settings for reading line (Python v2.6) removed and modified
#           with serial.Serial. Now code is working: before it read only once
#
########################################################
#Notes
#This code publishes the gps readings to node 4. 
#X corresponds to East
#Y corresponds to North
#From the original code the typo in the variable name: number_of_satellites has
#been corrected throughout the code as well as in position.msg.
#
#For Python v2.6+, the eol parameter for readline() is not longer supported! 
#This required some changes to the code by Enrico Anderlini 
#However, for clarity these changes have also been commented with double ##
#as above.
########################################################

import roslib; roslib.load_manifest('ASV')
import rospy
import serial
import time
import numpy
import math

import string
from pylab import *
from ASV.msg import position
from ASV.msg import status

################### GLOBAL VARIABLES ################### 

global serialPort
#global serialIO                               ##This addition is required with Python v2.6+
global identifier
global latitude                                           
global lat_NorS
global longitude                                          
global long_WorE
global time_gps
global number_of_satellites
global fix
global speed #in knots

################### SERIAL SETUP ################### 

def setUpSerial():
    global serialPort
    
    serialPort = serial.Serial(port='/dev/usbgps', baudrate='4800', timeout=0.1) # may need to change timeout if having issues!
    ##The line above needs changing with the new gps
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    
    print "Initialised GPS Serial."
    print serialPort.portstr
    print serialPort.isOpen()
    return serialPort.isOpen()

################### Functions to process different types of incoming message ###################  
#For more information check http://aprs.gids.nl/nmea/

def gpgga(string):
    global latitude
    global lat_NorS
    global longitude
    global long_WorE
    global time_gps
    global number_of_satellites              
    global fix

    try:
        time_gps  = float(string[1])
        latitude  = string[2]
        lat_NorS  = string[3]
        longitude = string[4]
        long_WorE = string[5]
        number_of_satellites = int(string[7]) 
	fix       = int(string[6])
    except: 
        pass

def gpgsa(string):
    pass  # a=0 #Incoming message contains details of satelites IDs.  This is currently of no use to us so will be ignored

def gprmc(string):
    global latitude
    global lat_NorS
    global longitude
    global long_WorE
    global time_gps
    global speed #in m/s

    try:
        time_gps  = float(string[1])
        latitude  = string[3]
        lat_NorS  = string[4]
        longitude = string[5]
        long_WorE = string[6]
        speed     = float(string[7])*0.514444444    # Convert from knots to m/s
    except:
        pass

    #magnetic_variation = float(string[10])		#Information doesnt appear to be received, perhaps a different type of GPS is needed
    #magvar_WorE = string[11]       

def gpgsv(string):
    global number_of_satellites                          
    if len(string[3]) == 1:
	number_of_satellites = int(string[3])            

################### MAIN LOOP ################### 

def validDataCheck():
    global serialPort
    global identifier
    attempt     = 0
    attempt_lim = 5

    while attempt < attempt_lim:
        while not serialPort.read(1) == '$':
            pass
        #The following line has been left for clarity of the change required with new Python versions
        ##data = serialPort.readline(size = None, eol = '\n')   #Do NOT uncomment this line!
        data = serialPort.readline()                        
        split_data = string.split(data,',')	#Split message by comma separator
        print 'DataCHECK: ', data

        message_identifier = split_data[0]	
        if message_identifier in identifier:
            return True
        else:
            if attempt >= attempt_lim:
                return False    
            else:
                attempt = attempt + 1                           ##or += attempt

def listenForData(status):

    #Establish Global Variables
    global	latitude
    global      lat_NorS
    global      longitude
    global      long_WorE
    global      time_gps	
    global      number_of_satellites                           
    global      fix
    global      speed

    #Initialise values to zero - important as different messages contain different values
    latitude		=	'0'	
    latitude_decimal	=	0
    lat_NorS		=	'0'
    longitude		=	'0'
    longitude_decimal	=	0
    long_WorE		=	'0'
    time_gps		=	0
    number_of_satellites=	0                               
    fix			=	0
    speed 		= 	0
    X			= 	0
    Y			= 	0
    
    try: 
        lat_orig = rospy.get_param('lat_orig')
        long_orig = rospy.get_param('long_orig')
    except:
        lat_orig = 51.0149116667
        long_orig = -1.49480166667

    first_reading = True
    mean_earth_radius = 6370973.27862					#metres
            
    while not rospy.is_shutdown():

        try:
            while serialPort.inWaiting() > 0 and serialPort.read(1) == '$': #while there is data to be read - read the line...
            
                pubStatus.publish(nodeID = 4, status = status)
                        
                ##data = serialPort.readline(size = None, eol = '\n')		#This has been left for clarity of the change required with new Python versions
                data = serialPort.readline() 
                split_data = string.split(data,',')				#Split message by comma separator
                #print 'Data: ', data

                message_identifier = split_data[0]				#Message identifier is the first part of the message and defines message format
                identifier[message_identifier](split_data)			#Process message according to indentifier type
                
                #### Function to convert latitude to decimal degrees ####
                if len(latitude) == 9:
                    minutes = numpy.float64(latitude[2:9])/60                   #numpy.float64 inserted instead of standard float
                    degrees = int(latitude[0:2])
                    latitude_decimal = degrees+minutes
                    if lat_NorS == 'S':
                        latitude_decimal = -latitude_decimal
                
                #### Function to convert longitude to decimal degrees ####
                if len(longitude) == 10:
                    minutes = numpy.float64(longitude[3:10])/60                 #numpy.float64 inserted instead of standard float
                    degrees = int(longitude[0:3])
                    longitude_decimal = degrees+minutes
                    if long_WorE == 'W':
                        longitude_decimal = -longitude_decimal

                #### Calculate X Y co-ordinates from original position ####
                try: #first_reading == False and len(latitude) == 9 and len(longitude) == 10:
                    Range=distanceTwoLatLong(lat_orig,latitude_decimal,long_orig,longitude_decimal)
                    Bearing=bearingTwoLatLong(lat_orig,latitude_decimal,long_orig,longitude_decimal)
                    BearingRad=math.radians(Bearing) 
                    X = Range*sin(BearingRad)
                    Y = Range*cos(BearingRad)
                except:
                    pass
                
                ##Check in order to understand read error
                print 'Lat: ', latitude_decimal
                print 'Long: ', longitude_decimal
                print 'Time: ', time_gps
                print 'No. sat.:', number_of_satellites
                print 'Fix:',   fix
                print 'Speed:', speed
                print 'x:', X
                print 'y:', Y
                #Publish data to position
                pub.publish(lat = latitude_decimal, long = longitude_decimal, time = time_gps, number_of_satellites = number_of_satellites, ValidGPSfix = fix, speed = speed, X = X, Y= Y)
                
                
        except:
            print 'read error'
        
        #print 'sleeping...'
        time.sleep(0.01)
        

def distanceTwoLatLong(lat1,lat2,lon1,lon2): #returns distance between two locations in lat/long in meters.
# Code from http://www.movable-type.co.uk/scripts/latlong.html
	R = 6371000 #Radius of the earth in m
	dLat = math.radians(lat2-lat1)
	dLon = math.radians(lon2-lon1)
	lat1 = math.radians(lat1)
	lat2 = math.radians(lat2)

	a = math.sin(dLat/2)*math.sin(dLat/2)+math.sin(dLon/2)*math.sin(dLon/2)*math.cos(lat1)*math.cos(lat2)
	c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a)) 
	d = R*c
	return d

def bearingTwoLatLong(lat1,lat2,lon1,lon2): #returns bearing between two locations in lat/long in degrees.
# Code from http://www.movable-type.co.uk/scripts/latlong.html
	dLat = math.radians(lat2-lat1)
	dLon = math.radians(lon2-lon1)
	lat1 = math.radians(lat1)
	lat2 = math.radians(lat2)
	y = math.sin(dLon) * math.cos(lat2)
	x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon);
	brng = math.atan2(y, x)
	brng= math.degrees(brng)

  	return brng



################### SHUTDOWN FUNCTION ################### 
def shutdown():
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 4, status = False)
    serialPort.close()

################### INITIALISING FUNCTION ################### 

if __name__ == '__main__':
    time.sleep(4) #Allow System to come Online    
    global identifier
    global pub    
    rospy.init_node('gps_sensor')
    rospy.on_shutdown(shutdown)         #Defining shutdown behaviour  
       
    identifier = {'GPGGA' : gpgga,
                'GPGSA' : gpgsa,
                'GPRMC' : gprmc,
                'GPGSV' : gpgsv,
    }
  
    #Define Publishers
    pubStatus = rospy.Publisher('status', status)
    pub = rospy.Publisher('position', position)           #changed from original, same in the rest of the code 
    time.sleep(1)
    #Setup serial port and check its status
    port_status = setUpSerial()    
    str = "GPS port status = %s. Port = %s" %(port_status, serialPort.portstr)
    rospy.loginfo(str)
    
    #Initial Data Test
    
    string_status=validDataCheck()
    
    if (port_status and string_status) == True:   
        status = True 
        pubStatus.publish(nodeID = 4, status = status)
        rospy.loginfo("GPS online")
    else:
        status = False
        pubStatus.publish(nodeID = 4, status = status)    


    listenForData(status)                     #Main loop for receiving data

    
