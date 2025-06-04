#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import MagneticField

def mag_callback(data):
    rospy.loginfo_throttle(5, "----- Dados do Magnetometro -----")
    rospy.loginfo_throttle(5, "Campo Magnetico (Tesla):")
    rospy.loginfo_throttle(5, "  x: {:.6f}, y: {:.6f}, z: {:.6f}".format(
        data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z))
    rospy.loginfo_throttle(5, "-----------------------------")

def mag_listener():
    rospy.init_node('mag_listener_node', anonymous=True)
    rospy.Subscriber('/mavros/imu/mag', MagneticField, mag_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        mag_listener()
    except rospy.ROSInterruptException:
        pass