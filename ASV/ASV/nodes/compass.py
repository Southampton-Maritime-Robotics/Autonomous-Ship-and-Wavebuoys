#!/usr/bin/python
##############################################################################
#compass.py
#
#Initial code is a simplified subset of compass_oceanserver.py from DelphinROSv2
#Code by Dr Alexander Brian Phillips, Leo Steenson and Catherine Harris
#Code modified and adapted for use in ASV package by Enrico Anderlini 
#(ea3g09@soton.ac.uk).
#
#
#Modifications to code
#11/12/2012  in order to account for correct installation in ASV BP. All 
#modifications are marked with ##
#04/02/2013  extensive comments added
#06/02/2013  calibration function removed and simplification of the code
#            after calibration of the compass (by Dr Alexander Brian Phillips).
#05/03/2013  io settings removed and changed with serial.Serial and readline().
#            Now code works fine. 
#
##############################################################################
#Notes
#
#This code publishes the compass readings to node ID 5. This code is designed 
#to operate with an Ocean Server compass.
#
#NOTES: For Python v2.6+, the eol parameter for readline() is no longer 
#supported! This required some changes to the code by Dr Alexander Brian 
#Phillips. For more information on this issue see: 
#http://pyserial.sourceforge.net/shortintro.html . However, for clarity these
#changes have also been commented with double ## as above. This has been changed
#by Enrico Anderlini, as the program read the data only once. Now problem has
#been fixed.
#
#The calibration of the compass has been performed according to the guide, since
#it is going to be installed in the correct orientantion. More information
#on this process is to be found at page 17 of the OS5000 family guide that
#can be downloaded from http://www.oceanserver-store.com/compass.html.
#This process is to be performed in  Cutecom, which is in hexadecimal language.
#Therefore, for calibration on a level plate, the required commands are:
#ascii           hexadecimal
#Esc + C  -->    1B + 43
#Tab      -->     20                  to move to the other commmand
#For caliration of the rolled sensor:
#Esc + Z  -->    1B + 5A
#Tab      -->     20                  to move to the other commmand
#
##############################################################################
import roslib; roslib.load_manifest('ASV')    ##
import rospy
import numpy
import serial
import time
import math
from re import findall
from ASV.msg import compass                   
from ASV.msg import status                    
global serialPort


################################################################
#The following function sets up and opens the serial port of the compass.
#This port has been set in the udev local rules of the computer in the file
#/etc/udev/rules.d/10-local.rules as /dev/usbcompass.
def setUpSerial():
    global serialPort
    
    #Specifying the serial port to be opened
    serialPort = serial.Serial(port='/dev/usbcompass', baudrate='115200', timeout=0.01) # may need to change timeout if having issues!
    ##in the line above /dev/usboceanserver has been  substituted according to the settings in the local rules.
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.stopbits = serial.STOPBITS_ONE
    serialPort.parity = serial.PARITY_NONE
    
    print "Initialised OceanServer serial."
    print serialPort.portstr
    print serialPort.isOpen()
    serialPort.flushInput()
    serialPort.flushOutput()
    return serialPort.isOpen()
    
    
################################################################
#The following function reads the data being published by the serial port.
def listenForData(status):
    global serialPort
    
    time_zero = time.time()
    
    ####PTo be used in the Pitch Filter#####
    #pitch_array_length = 10
    #Px = numpy.zeros([pitch_array_length],float)
    #Py = numpy.zeros([pitch_array_length],float)
    #for i in range(0,pitch_array_length): 
    #    Px[i] = i
    
    #####################   
    
    
    while not rospy.is_shutdown():    
        try:
            time.sleep(0.01)  # Prevents node from using 100% CPU!!

            while serialPort.inWaiting() > 0 and serialPort.read(1) == '$':     #while there is data to be read - read the line...
                
                pubStatus.publish(nodeID = 5, status = status)
                
                dataRaw = serialPort.readline() 
                
                print dataRaw
                                
                data = numpy.array((findall('[-]*\d+.\d+',dataRaw)), numpy.float)
                
                try:

                    dt = time.time() - time_zero
                    #print data
                    time_zero = time.time()
                    
                    heading     = data[0]
                    pitch       = data[1]
                    roll        = data[2]    
                    temperature = data[3]          
                    depth       = data[4]
                    m           = data[5]
                    mx          = data[6]
                    my          = data[7]
                    mz          = data[8]
                    a           = data[9]
                    ax          = data[10]
                    ay          = data[11]
                    az          = data[12]
                    
                    
                    
                    #The following print statements should be left uncommented only during debugging                    
                    print '*******'
                    print 'heading %f' %(heading)
                    print 'pitch  %f' %(pitch)
                    print 'roll %f' %(roll)
                    print 'temperature %f' %(temperature)
                    #print 'm %f'  %(m)                              ##
                    #print 'mx %f' %(mx)                             ##
                    #print 'my %f' %(my)                             ##
                    #print 'mz %f' %(mz)                             ##
                    #print 'a %f'  %(a)                              ##
                    #print 'ax %f' %(ax)                             ##
                    #print 'ay %f' %(ay)                             ##
                    #print 'az %f' %(az)                             ##
     
                    #Publish data to compass_out
                    pub.publish(heading=heading,pitch=pitch,roll=roll,temperature=temperature,m=m,mx=mx,my=my,mz=mz,a=a,ax=ax,ay=ay,az=az)
                    
                except ValueError: 
                    print 'not a float'
                
        except:
            print 'read error'
        
        

def validDataCheck():
    attempts = 1
    
    while attempts < 5:
        
        while not serialPort.read(1) == '$':
            pass
        #The following line has been left for clarity of the change required with new Python versions
        #dataRaw = serialPort.readline(size = None, eol = '\n')   #This line MUST be left commented!
        #The following line contains the actual change required with Python v2.6+; the command reads in line of the data
        dataRaw = serialPort.readline()  #serialIO.readline()
        data = findall('[-]*\d+.\d+',dataRaw)
        
        #Uncomment the following line only during debugging
        #print data
        if len(data) == 13:
            return True
    
    return False
################################################################
#Shut down function, which closes the serial port
def shutdown():
    serialPort.flushInput()
    serialPort.flushOutput()
    pubStatus.publish(nodeID = 5, status = False)
    serialPort.close()

################################################################        
#     INITIALISE     ###########################################
################################################################

if __name__ == '__main__':
    time.sleep(1) #Allow System to come Online    
    rospy.init_node('OceanServer_compass')
    
    global pub
    global serialPort
    
    ####Setting up the publisher####
    pub = rospy.Publisher('compass_out', compass)   
    pubStatus = rospy.Publisher('status', status)
    
    rospy.on_shutdown(shutdown)  
    
    port_status = setUpSerial()
    str = "OceanServer port status = %s. Port = %s" %(port_status, serialPort.portstr)
    rospy.loginfo(str)
    time.sleep(0.3)
    string_status=validDataCheck()
    
    if (port_status and string_status) == True:    
        status = True
        #Publishing the status of the node
        pubStatus.publish(nodeID = 5, status = status)
        rospy.loginfo("OceanServer online")
    else:
        #Publishing the status of the node
        status = False
        pubStatus.publish(nodeID = 5, status = status)
      
    
    listenForData(status)   #Main loop for receiving data
    
    
