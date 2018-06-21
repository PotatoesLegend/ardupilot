#!/usr/bin/env python

# Tao Du
# taodu@csail.mit.edu
# Jun 21, 2018
# A rosnode that fakes the Vicon data at 50Hz.

import roslib; roslib.load_manifest('roscopter')
import rospy
from geometry_msgs.msg import Vector3
from mit_msgs.msg import MocapPosition

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('copter', MocapPosition, queue_size=50)
        rospy.init_node('fakevicon')
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            pub.publish('fake vicon data', 0.0, Vector3(4000, 2000, 1000), Vector3(1.0, 2.0, 4.0))
            rate.sleep()
    except rospy.ROSInterruptException:
        raise ValueError('Failed to execute simulate_vicon_topic.py') 
