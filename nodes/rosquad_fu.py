#!/usr/bin/env python

#import roslib; roslib.load_manifest('roscopter')
import rospy
import serial
from pymavlink import mavutil

from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry

import sys,struct,time,os
from std_srvs.srv import *

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))

from optparse import OptionParser
parser = OptionParser("rosquad.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
parser.add_option("--device", dest="device", default="/dev/ttyACM0", help="serial device")
parser.add_option("--rate", dest="rate", default=10, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-control",dest="enable_control", default=False, help="Enable listning to control messages")

(opts, args) = parser.parse_args()

# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def sendPosition(data):	
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    master.mav.optical_flow_send(0,0,0,0,x,y,255,z)

    rospy.loginfo("Received data is %f, %f, %f" ,x,y,z);

def mainloop():
    rospy.init_node('rosquad')
    global start
    global x, y, z
    start = 1
    while not rospy.is_shutdown():
        rospy.Subscriber("visual_odometry", Odometry, sendPosition)
        rospy.spin() 

if __name__ == '__main__':
    try:
        mainloop()

    except rospy.ROSInterruptException: pass
