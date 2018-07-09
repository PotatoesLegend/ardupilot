#!/usr/bin/env python
import os
import sys
import struct
import time

import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from mit_msgs.msg import MocapPosition

import roscopter.msg
from derivative_filter import *
from transformations import *

mavlink_dir = os.path.realpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '..', 'mavlink'))
sys.path.insert(0, mavlink_dir)

pymavlink_dir = os.path.join(mavlink_dir, 'pymavlink')
sys.path.insert(0, pymavlink_dir)


from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
parser.add_option("--device", dest="device", default="/dev/ttyUSB0", help="serial device")
parser.add_option("--rate", dest="rate", default=10, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-control",dest="enable_control", default=False, help="Enable listning to control messages")
parser.add_option("--enable-vicon", dest="enable_vicon", default=False, help="Enable listening to vicon data")
parser.add_option("--vicon-name", dest="vicon_name", type='string', help="Name of the rostopic that publishes Vicon data")
parser.add_option("--vicon-rate", dest="vicon_rate", type='int', help="Control the frequency of vicon publisher", default=25)

(opts, args) = parser.parse_args()

import mavutil

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


def send_rc(data):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        data.channel[0],
        data.channel[1],
        data.channel[2],
        data.channel[3],
        data.channel[4],
        data.channel[5],
        data.channel[6],
        data.channel[7])
    print "sending rc: %s" % data

# Time stamp in seconds.
init_time_stamp = time.time()
last_time_offset = 0
vx = OneSideSmoothFilter(5)
vy = OneSideSmoothFilter(5)
vz = OneSideSmoothFilter(5)
vroll = OneSideSmoothFilter(5)
vpitch = OneSideSmoothFilter(5)
vyaw = OneSideSmoothFilter(5)
last_yaw = np.nan
def send_vicon(data):
    # Forwards Vicon data to the board.
    # Unit in data: mm for location, rad for angles.
    # Unit to send to board: mm for location, rad for angles.
    pos = data.translational
    x, y, z = pos.x, pos.y, pos.z
    # Unfortunately the inertia frame in the simulator is not the same as in
    # VICON, so we have to convert x, y, z manually. Specifically, in VICON
    # we set x to right, y to front, and z to up. In our simulator, however,
    # x is front, y is right, and z is down.
    x, y, z = y, x, -z

    # Get the rotational angle.
    rpy = data.axisangle
    ax, ay, az = rpy.x, rpy.y, rpy.z
    theta = math.sqrt(ax * ax + ay * ay + az * az)
    # Get the rotational axis. If theta is close to zero, set the axis to
    # [0, 0, 1] and theta to 0.
    if math.fabs(theta) < 1e-5:
        theta = 0.0
        ax = 0.0
        ay = 0.0
        az = 1.0
    else:
        ax = ax / theta
        ay = ay / theta
        az = az / theta
    # Note that (ax, ay, az) is defined in the WORLD frame, and if we rotate
    # the WORLD frame along (ax, ay, az) by theta, we will get our BODY frame.
    # Now here is a discrepancy: we would like to switch our WORLD frame to
    # North(x)-East(y)-Down(z) frame so that it is aligned with our BODY frame.
    R = rotation_matrix(theta, [ax, ay, az])
    # Now each column in R represents the corresponding axis of the BODY frame
    # in the WORLD frame.
    R2 = numpy.dot(numpy.array([[0.0, 1.0, 0.0, 0.0],
                               [1.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, -1.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0]]), R)
    # Now R represents the transformations between the BODY frame and the faked
    # North-East-Down frame. Specifically, R * [1 0 0 0]' returns the x axis of
    # the BODY frame in the NED frame. Now if we solve the Euler angles from R
    # we should be able to get the faked yaw angle. All angles are in radians.
    roll, pitch, yaw = euler_from_matrix(R2, 'sxyz')
    # Uncomment the following lines to double check euler angles.
    #Rroll = rotation_matrix(roll, [1.0, 0.0, 0.0])
    #Rpitch = rotation_matrix(pitch, [0.0, 1.0, 0.0])
    #Ryaw = rotation_matrix(yaw, [0.0, 0.0, 1.0])
    #if not numpy.allclose(R2, numpy.dot(numpy.dot(Ryaw, Rpitch), Rroll)):
    #    print('Euler angles are wrong!')

    new_to = time.time() - init_time_stamp
    global last_time_offset
    if new_to - last_time_offset > 1.0 / opts.vicon_rate:
        # Check if we need to round yaw.
        global last_yaw
        if not np.isnan(last_yaw):
            dy = yaw - last_yaw
            # Find among all y + 2kpi the closest one to 0.
            dy /= 2.0 * np.pi
            # Now find among all y + k the closest one to 0.
            yl = np.floor(dy)
            yc = np.ceil(dy)
            ks = np.array([-yl, -yc, -yl - 1, -yc + 1])
            ys = yaw + ks * 2.0 * np.pi
            yaw = ys[np.argmin(np.abs(ys - last_yaw))]
        last_yaw = yaw
        # Compute linear and angular velocities.
        # Velocities are in mm/s and rad/s
        vx.update(new_to, x)
        vy.update(new_to, y)
        vz.update(new_to, z)
        vroll.update(new_to, roll) 
        vpitch.update(new_to, pitch) 
        vyaw.update(new_to, yaw) 
        last_time_offset = new_to
        master.mav.vicon_send(x, y, z, roll, pitch, yaw, \
            vx.slope(), vy.slope(), vz.slope(), vroll.slope(), vpitch.slope(), vyaw.slope())

def set_arm(req):
    master.arducopter_arm()
    return []

def set_disarm(req):
    master.arducopter_disarm()
    return []

# ROS recommends using nonzero queue_size.
queue_size = 10
pub_gps = rospy.Publisher('gps', NavSatFix, queue_size=queue_size)
pub_rc = rospy.Publisher('rc', roscopter.msg.RC, queue_size=queue_size)
pub_state = rospy.Publisher('state', roscopter.msg.State, queue_size=queue_size)
pub_vfr_hud = rospy.Publisher('vfr_hud', roscopter.msg.VFR_HUD, queue_size=queue_size)
pub_attitude = rospy.Publisher('attitude', roscopter.msg.Attitude, queue_size=queue_size)
pub_raw_imu =  rospy.Publisher('raw_imu', roscopter.msg.Mavlink_RAW_IMU, queue_size=queue_size)
if opts.enable_control:
    rospy.Subscriber('send_rc', roscopter.msg.RC , send_rc)
if opts.enable_vicon:
    rospy.Subscriber(opts.vicon_name, MocapPosition, send_vicon)

#define service callbacks
arm_service = rospy.Service('arm', Empty, set_arm)
disarm_service = rospy.Service('disarm', Empty, set_disarm)

#state
gps_msg = NavSatFix()

def mainloop():
    rospy.init_node('roscopter')
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        msg = master.recv_match(blocking=False)
        if not msg:
            continue
        #print msg.get_type()
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else: 
            msg_type = msg.get_type()
            if msg_type == "RC_CHANNELS_RAW" :
                pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]) 
            if msg_type == "HEARTBEAT":
                pub_state.publish(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 
                                  msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 
                                  mavutil.mode_string_v10(msg))
            if msg_type == "VFR_HUD":
                pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)

            if msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >=3:
                    fix=NavSatStatus.STATUS_FIX
                pub_gps.publish(NavSatFix(latitude = msg.lat/1e07,
                                          longitude = msg.lon/1e07,
                                          altitude = msg.alt/1e03,
                                          status = NavSatStatus(status=fix, service = NavSatStatus.SERVICE_GPS) 
                                          ))
            #pub.publish(String("MSG: %s"%msg))
            if msg_type == "ATTITUDE" :
                pub_attitude.publish(msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)


            if msg_type == "LOCAL_POSITION_NED" :
                print "Local Pos: (%f %f %f) , (%f %f %f)" %(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)

            if msg_type == "RAW_IMU" :
                pub_raw_imu.publish (Header(), msg.time_usec, 
                                     msg.xacc, msg.yacc, msg.zacc, 
                                     msg.xgyro, msg.ygyro, msg.zgyro,
                                     msg.xmag, msg.ymag, msg.zmag)


wait_heartbeat(master)

print("Sending all stream request for rate %u" % opts.rate)
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    opts.rate,
    1)

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException: pass
