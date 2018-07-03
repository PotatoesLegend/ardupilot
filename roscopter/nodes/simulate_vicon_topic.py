#!/usr/bin/env python

# Tao Du
# taodu@csail.mit.edu
# Jun 21, 2018
# A rosnode that fakes the Vicon data at 50Hz.

import time
import math
import roslib; roslib.load_manifest('roscopter')
import rospy
from geometry_msgs.msg import Vector3
from mit_msgs.msg import MocapPosition
from optparse import OptionParser

parser = OptionParser("simulate_vicon_topic.py [options]")

parser.add_option("--type", dest="type", type='str', help='const or sin', default='const')
(opts, args) = parser.parse_args()
curve_type = opts.type
if curve_type not in ['const', 'sin']:
    print("Unsupported curve type.")
    sys.exit(-1)

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('copter', MocapPosition, queue_size=50)
        rospy.init_node('fakevicon')
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if curve_type == 'const':
                x, y, z = 4000, 2000, 1000
                roll, pitch, yaw = 1.0, 2.0, 4.0
            elif curve_type == 'sin':
                t = time.time()
                omega = math.pi      
                x = 4000 * math.sin(omega * t)
                y = 2000 * math.sin(omega * t + math.pi / 2)
                z = 1000 * math.sin(omega * t + math.pi)
                roll = 1.0 * math.sin(omega * t)
                pitch = 2.0 * math.sin(omega * t + math.pi / 2)
                yaw = 4.0 * math.sin(omega * t + math.pi)
            else:
                # Should never happen.
                raise ValueError('Should not happen.')
            pub.publish('fake vicon data', 0.0, Vector3(x, y, z), Vector3(roll, pitch, yaw))
            rate.sleep()
    except rospy.ROSInterruptException:
        raise ValueError('Failed to execute simulate_vicon_topic.py') 
