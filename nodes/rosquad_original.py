#!/usr/bin/env python
import roslib; roslib.load_manifest('roscopter')
import rospy
import serial

from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import roscopter.msg
import sys,struct,time,os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))

from optparse import OptionParser
from pymavlink import mavutil

parser = OptionParser("rosquad.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
parser.add_option("--device", dest="device", default="/dev/ttyACM0", help="serial device")
parser.add_option("--rate", dest="rate", default=10, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-control",dest="enable_control", default=False, help="Enable listning to control messages")

(opts, args) = parser.parse_args()

# Custom mode definitions from PX4 code base
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_SEATBELT = 2
PX4_CUSTOM_MAIN_MODE_EASY = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4

PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6

# mavlink base_mode flags
MAV_MODE_FLAG_DECODE_POSITION_SAFETY = 0b10000000
MAV_MODE_FLAG_DECODE_POSITION_MANUAL = 0b01000000
MAV_MODE_FLAG_DECODE_POSITION_HIL = 0b00100000
MAV_MODE_FLAG_DECODE_POSITION_STABILIZE = 0b00010000
MAV_MODE_FLAG_DECODE_POSITION_GUIDED = 0b00001000
MAV_MODE_FLAG_DECODE_POSITION_AUTO = 0b00000100
MAV_MODE_FLAG_DECODE_POSITION_TEST = 0b00000010
MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 0b00000001

# masks to prevent accidental arm/disarm of motors
DISARM_MASK = 0b01111111
ARMED_MASK = 0b10000000

custom_mode = { 'main_mode' : 1, 'sub_mode' : 0 }
base_mode = 81

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

def custom_mode_value(sub_mode, main_mode):
    '''build the 32-bit custom_mode value from the sub and main modes'''
    return ((sub_mode << 8) | main_mode) << 16

def base_mode_value(new_custom_mode):
    '''build the 8-bit base_mode value'''
    global base_mode
    if new_custom_mode == PX4_CUSTOM_MAIN_MODE_AUTO:
        return (base_mode & ARMED_MASK | MAV_MODE_FLAG_DECODE_POSITION_STABILIZE | MAV_MODE_FLAG_DECODE_POSITION_GUIDED | MAV_MODE_FLAG_DECODE_POSITION_AUTO | MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE) #_0011101
    else:
        return (base_mode & ARMED_MASK | MAV_MODE_FLAG_DECODE_POSITION_MANUAL | MAV_MODE_FLAG_DECODE_POSITION_STABILIZE | MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE) #_1010001
        
def px4_arm():
    '''Arm the Pixhawk'''
    global custom_mode
    global base_mode
    #print(custom_mode)
    master.mav.set_mode_send(master.target_system, base_mode | MAV_MODE_FLAG_DECODE_POSITION_SAFETY, custom_mode_value(custom_mode['sub_mode'], custom_mode['main_mode']))
    print "Pixhawk armed"

def px4_disarm():
    '''Disarm the Pixhawk'''
    global custom_mode
    global base_mode
    master.mav.set_mode_send(master.target_system, base_mode & DISARM_MASK, custom_mode_value(custom_mode['sub_mode'], custom_mode['main_mode']))
    print "Pixhawk disarmed"

def px4_auto(req):
	'''Set Pixhawk to AUTO mode'''
	global base_mode
	print("Mode calculated to be %d" %custom_mode_value(PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF, PX4_CUSTOM_MAIN_MODE_AUTO))
	master.mav.set_mode_send(master.target_system, base_mode_value(PX4_CUSTOM_MAIN_MODE_AUTO), custom_mode_value(PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF, PX4_CUSTOM_MAIN_MODE_AUTO))
	print "Pixhawk is now in AUTO mode"

def px4_easy():
	'''Set Pixhawk to EASY mode'''
	global base_mode
	master.mav.set_mode_send(master.target_system, base_mode_value(PX4_CUSTOM_MAIN_MODE_EASY), custom_mode_value(PX4_CUSTOM_MAIN_MODE_EASY, PX4_CUSTOM_MAIN_MODE_EASY))

def px4_takeoff(req):
	print("Sending command to mavlink")
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 6000)
	rospy.sleep(2.)
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 12000)
	rospy.sleep(2.)
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 18000)
	rospy.sleep(2.)
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 24000)
	rospy.sleep(2.)
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 28000)
	rospy.sleep(2.)

def px4_land(req):
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 18000)
	rospy.sleep(2.)
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 12000)
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 6000)
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, 2000)

def px4_moveforward(req):
	'''Start moving forward'''
	print("Moving forward")
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, -32767, 0, 0)

def px4_movebackward(req):
	'''Start moving forward'''
	print("Moving backward")
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 32767, 0, 0)

def px4_moveright(req):
	'''Start moving forward'''
	print("Moving right")
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 32767, 0, 0, 0)

def px4_moveleft(req):
	'''Start moving forward'''
	print("Moving left")
	master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, -32767, 0, 0, 0)

def find_point(data):
    global throttlecmd
    throttlecmd = 0
    if data.ns == "globalArrow":
        pointx = data.points[1].x;
        pointy = data.points[1].y;
        pointz = data.points[1].z;
        print "Goal position is : %f, %f, %f" %(pointx, pointy, pointz)

	#msg = master.recv_match(blocking=False)
    #msg_type = msg.get_type()
	#if msg_type == "RC_CHANNELS_RAW" :
    #	pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]) 
	#	print "RC throttle is %d" % msg.chan3_raw
	#	throttle = msg.chan3_raw
	#        throttlecmd = (throttle - 1170) / 700
	#	throttlecmd = throttlecmd * 32767

	if pointz > 1:
	    rospy.loginfo("Moving forward")
            master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, -32767, 0, throttlecmd)
            #px4_moveforward()

	if pointz < 1:
            rospy.loginfo("Reached point")
	    master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, throttlecmd)
	    #px4_movebackward()

	

pub_gps = rospy.Publisher('gps', NavSatFix)
#pub_imu = rospy.Publisher('imu', Imu)
pub_rc = rospy.Publisher('rc', roscopter.msg.RC)
pub_state = rospy.Publisher('state', roscopter.msg.State)
pub_vfr_hud = rospy.Publisher('vfr_hud', roscopter.msg.VFR_HUD)
pub_attitude = rospy.Publisher('attitude', roscopter.msg.Attitude)
pub_raw_imu =  rospy.Publisher('raw_imu', roscopter.msg.Mavlink_RAW_IMU)
if opts.enable_control:
    #rospy.Subscriber("control", roscopter.msg.Control , mav_control)
    rospy.Subscriber("send_rc", roscopter.msg.RC , send_rc)

#define service callbacks
arm_service = rospy.Service('arm',Empty,px4_arm)
disarm_service = rospy.Service('disarm',Empty,px4_disarm)
auto_service = rospy.Service('auto',Empty,px4_auto)
forward_service = rospy.Service('forward',Empty,px4_moveforward)
takeoff_service = rospy.Service('takeoff',Empty,px4_takeoff)
land_service = rospy.Service('land',Empty,px4_land)

#state
gps_msg = NavSatFix()
    
def mainloop():
    rospy.init_node('rosquad')
    while not rospy.is_shutdown():
        rospy.sleep(0.001)

    ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
    print ser.name
    ser.write(chr(0x0d))
    ser.write(chr(0x0d))
    ser.write(chr(0x0d))
    ser.write("mavlink_offboard start -d /dev/ttyACM0 -b 230400")
    ser.write(chr(0x0d))
    ser.write(chr(0x0d))
    ser.write(chr(0x0d))
    ser.write("nshterm stop")
    ser.write(chr(0x0d))
    ser.write(chr(0x0d))
    ser.write(chr(0x0d))
    ser.close()

    print "Listening"
    rospy.Subscriber("visualization_doors", Marker, find_point)
    rospy.spin() 
    throttlecmd = 0
    master.mav.set_quad_swarm_roll_pitch_yaw_thrust_send(1, 2, 0, 0, 0, throttlecmd)
	
    msg = master.recv_match(blocking=False)
    print "Continuing"

# wait for the heartbeat msg to find the system ID
# wait_heartbeat(master)

# waiting for 10 seconds for the system to be ready
print("Getting system ready.. please wait.")

print("Sending all stream request for rate %u" % opts.rate)
#for i in range(0, 3):

master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)

#master.mav.set_mode_send(master.target_system, 
if __name__ == '__main__':
    try:
        mainloop()

    except rospy.ROSInterruptException: pass
