#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import FluidPressure

def pressure_callback(data):
    rospy.loginfo_throttle(5, "----- Dados de Pressao Atmosferica -----")
    rospy.loginfo_throttle(5, "Pressao do Fluido (Pascal): {:.2f}".format(data.fluid_pressure))
    rospy.loginfo_throttle(5, "Variancia: {:.2f}".format(data.variance))
    rospy.loginfo_throttle(5, "------------------------------------")

def pressure_listener():
    rospy.init_node('pressure_listener_node', anonymous=True)
    rospy.Subscriber('/mavros/imu/atm_pressure', FluidPressure, pressure_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        pressure_listener()
    except rospy.ROSInterruptException:
        pass