#!/usr/bin/env python

import rospy
import serial
from pymavlink import mavutil

from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, Joy

import sys,struct,time,os
from std_srvs.srv import *

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))

from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

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

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def find_point(data):
    if data.ns == "globalArrow":
	pointx = data.points[1].x
	pointz = data.points[1].z
	print "Goal position is %f, %f" %(pointx, pointz)
	if abs(pointx) > 0.1: 
	    if pointx > 0:
	        print "         "
	        print "          \  "
	        print "___________\ "
	        print "           /"
	        print "          /"
	        print "            "	
	
                master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 10000, 0)
		#rospy.sleep(1)
		#master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 0)
		#rospy.sleep(2)
	        print "Stopping"   
		 

	    if pointx < 0:
		print "Going left"
		master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, -10000, 0)
		#rospy.sleep(1)
		#master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 0)
		#rospy.sleep(2)
		#rospy.sleep(1.5)

	elif abs(pointx) = 1000:
	    print "Avoiding wall"
	    master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, (pointx/abs(pointx))*32767, 0, 0, 0)

	else:
	    if pointz > 0.5:
		print "   /|\   "
		print "  / | \  "
		print " /  |  \ "
		print "/   |   \ "
		print "    |    "
		print "    |    "
		print "    |    "
		print "    |    "
		master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, -32767, 0, 0)

	    elif pointz < -8:
		print "    |    "
		print "    |    "
		print "    |    "
		print "\   |   /"
		print " \  |  / "
		print "  \ | /  "
		print "   \|/   "
		master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 32767, 0, 0)

def mainloop():
    rospy.init_node('roscopter')
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

	print "Listening"
    	rospy.Subscriber("visualization_doors", Marker, find_point)   
	rospy.spin()	

# wait for the heartbeat msg to find the system ID
# wait_heartbeat(master)

# waiting for 10 seconds for the system to be ready
print("Getting system ready.. please wait.")

print("Sending all stream request for rate %u" % opts.rate)
#for i in range(0, 3):

master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)

#master.mav.set_mode_send(master.target_system, 
if __name__ == '__main__':
    try:
        mainloop()

    except rospy.ROSInterruptException: pass
