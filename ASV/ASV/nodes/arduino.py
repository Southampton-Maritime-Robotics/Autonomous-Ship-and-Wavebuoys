#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#Notes:
#
#Original code name: serial_node.py within the package rosserial_python/nodes
#Incorporated (copied and pasted) within the package ASV by Enrico Anderlini
#in order to employ the rosserial_arduino capabilities for running ros nodes,
#including publishers and subscribers on the Arduinos used in the project.
#
#02/02/2013 Code copied into ASV package and modified in order to run from
#the actual arduino port: /dev/ttyACM0-may be modified if the port name is 
#changed in the udev rules.
#

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import roslib; roslib.load_manifest("rosserial_python")
import rospy
from rosserial_python import SerialClient, RosSerialServer
import multiprocessing

import sys

from library_highlevel import library_highlevel

global pubStatus
 
        

def shutdown(args=None):

    lib = library_highlevel()
    lib.stop()
    pubStatus.publish(nodeID = 1, status = False)
    serialPort.close()
    rospy.loginfo("Shutting down process %r", process)
    process.terminate()
    process.join()
    rospy.loginfo("All done")
    

if __name__=="__main__":

    port_name = rospy.get_param('~port','/dev/usbarduino')  #Modified from the original code, see udev local rules
    baud = int(rospy.get_param('~baud','57600'))         #This is for an Arduino Uno.
    tcp_portnum = int(rospy.get_param('/rosserial_embeddedlinux/tcp_port', '11411'))
    fork_server = rospy.get_param('/rosserial_embeddedlinux/fork_server', True)

    sys.argv = rospy.myargv(argv=sys.argv) 
    #import pdb; pdb.set_trace()
    
    if len(sys.argv) == 2 :
        port_name  = sys.argv[1]
    if len(sys.argv) == 3 :
        tcp_portnum = int(sys.argv[2])
    
    if port_name == "tcp" :
        server = RosSerialServer(tcp_portnum, fork_server)
        rospy.loginfo("Waiting for socket connections on port %d" % tcp_portnum)
        try:
            server.listen()
        except KeyboardInterrupt:
            rospy.loginfo("got keyboard interrupt")
        finally:
            rospy.loginfo("Shutting down")
            for process in multiprocessing.active_children():
                 serialPort.close()
                 rospy.loginfo("Shutting down process %r", process)
                 process.terminate()
                 process.join()
                 rospy.loginfo("All done")

    else :          # Use serial port 
        rospy.init_node("serial_node")
        rospy.loginfo("ROS Serial Python Node")
        rospy.loginfo("Connected on %s at %d baud" % (port_name,baud) )
        client = SerialClient(port_name, baud)
        try:
            client.run()
        except KeyboardInterrupt:
            pass
        
    rospy.on_shutdown(shutdown)

