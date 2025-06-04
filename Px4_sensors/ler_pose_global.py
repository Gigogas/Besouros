#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def global_pos_callback(data):
    rospy.loginfo_throttle(5, "----- Posicao Global Estimada -----")
    rospy.loginfo_throttle(5, "Status: {}".format(data.status.status))
    rospy.loginfo_throttle(5, "Latitude: {:.7f}".format(data.latitude))
    rospy.loginfo_throttle(5, "Longitude: {:.7f}".format(data.longitude))
    rospy.loginfo_throttle(5, "Altitude (WGS84): {:.2f} m".format(data.altitude))
    rospy.loginfo_throttle(5, "---------------------------------")

def global_pos_listener():
    rospy.init_node('global_pos_listener_node', anonymous=True)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, global_pos_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        global_pos_listener()
    except rospy.ROSInterruptException:
        pass