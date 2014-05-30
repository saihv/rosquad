#!/usr/bin/env python

import roslib; roslib.load_manifest('roscopter')
import rospy
import serial
from pymavlink import mavutil
import time

from std_msgs.msg import String, Header
from sensor_msgs.msg import Joy
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

global requirePID, prevtime, currtime;
prevtime = 0;

requirePID = 0

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" %(m.target_system, m.target_system))

def PID():
    global currtime, prevtime, Der, Int;
    currtime = time.time();
    dt = currtime - prevtime;
    sp = 0;
    msg = master.recv_match(blocking=False)
    if not msg:
        #while not msg:
	msg = master.recv_match(blocking=False)
	#    if msg.get_type() == "OPTICAL FLOW":
	#        vx = msg.flow_comp_m_x;
	#	vy = msg.flow_comp_m_y;
	#        break;
    vx = msg.flow_comp_m_x;
    vy = msg.flow_comp_m_y;

    error = sp-vx;

    Der = 0;
    Int = 0;
    
    Kp = 5;
    Kd = 0;
    Ki = 0;
    while abs(error) > 0.02:
        msg = master.recv_match(blocking=False)
	if not msg:
	    continue
	if msg.get_type() == "OPTICAL_FLOW":
	    vx = msg.flow_comp_m_x;
	    vy = msg.flow_comp_m_y;

	error = sp-vx;
	P = Kp * error;

	D = Kd * (error - Der)/dt;
	Der = error;

	Int += error*dt;
	I = Ki * Int;

	speed = P + I + D;

	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, speed*-32767, 0, 0)
	print "Sent %f"%speed

	prevtime = time.time();
	
def send_command(data):
    global requirePID
    speed = data.axes[4];
    print "Received velocity command is %f"%speed;

    master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, speed*-32767, 0, 0)

    if requirePID == 0:
	if abs(speed) > 0.5:
	    requirePID = 1

    if requirePID == 1:
	if abs(speed) <= 0.1:
            PID()
	    requirePID = 0

def mainloop():
    rospy.init_node('rosquad')
    while not rospy.is_shutdown():
    	#print "Listening"
	rospy.sleep(0.1)
	#master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, -32767, 0, 0)
	
    	rospy.Subscriber("joy", Joy, send_command)
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
