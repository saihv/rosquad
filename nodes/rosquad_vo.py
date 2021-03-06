#!/usr/bin/env python

import roslib; roslib.load_manifest('rosquad')
import rospy
import serial
from pymavlink import mavutil
import time

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

i = 1

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" %(m.target_system, m.target_system))

def send_command(data):
    currentx = data.pose.pose.position.z;
    currenty = data.pose.pose.position.x;
    currentz = data.pose.pose.position.y;
    global i
    global X
    global Y
    
    if i < 5:
	print "Target values are %f, %f"%(X[i],Y[i])
	print "Current values are %f, %f"%(currentx, currenty)
	
	if X[i] != X[i-1]:
	    diffx = (X[i]-X[i-1])/abs(X[i]-X[i-1])
	    if round(currentx) != X[i]:
	    	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(2, 1, 0, -1*diffx*24000, 0, 0)
		print "Moving to target"

	    else: 
	        master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(2, 1, 0, 0, 0, 0)
	        print "Stopping"
	        rospy.sleep(1.5)
		i = i+1

	elif Y[i] != Y[i-1]:
	    diffy = (Y[i]-Y[i-1])/abs(Y[i]-Y[i-1])
	    if round(currenty) != Y[i]:
		master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(2, 1, -1*diffy*24000, 0, 0, 0)
		print "Moving to target"

            else: 
	        master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(2, 1, 0, 0, 0, 0)
	        print "Stopping"
	        rospy.sleep(1.5)
		i = i+1

def mainloop():
    rospy.init_node('rosquad')
    while not rospy.is_shutdown():
    	rospy.sleep(0.1)
	global i
	global X
	global Y
	X = [0, 3, 3, 0, 0]
	Y = [0, 0, 2, 2, 0]
	rospy.Subscriber("cam2_to_init", Odometry, send_command)
    	rospy.spin()
        
# wait for the heartbeat msg to find the system ID
#wait_heartbeat(master)

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
