#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def rel_alt_callback(data):
    rospy.loginfo_throttle(5, "----- Altitude Relativa -----")
    rospy.loginfo_throttle(5, "Altitude Relativa (metros): {:.2f}".format(data.data))
    rospy.loginfo_throttle(5, "---------------------------")

def rel_alt_listener():
    rospy.init_node('rel_alt_listener_node', anonymous=True)
    rospy.Subscriber('/mavros/global_position/rel_alt', Float64, rel_alt_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        rel_alt_listener()
    except rospy.ROSInterruptException:
        pass