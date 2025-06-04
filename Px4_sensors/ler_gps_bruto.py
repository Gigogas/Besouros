#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def gps_raw_callback(data):
    rospy.loginfo_throttle(5, "----- Dados Brutos do GPS -----")
    rospy.loginfo_throttle(5, "Status: {}".format(data.status.status)) # consultar sensor_msgs/NavSatStatus para ver o status
    rospy.loginfo_throttle(5, "Latitude: {:.7f}".format(data.latitude))
    rospy.loginfo_throttle(5, "Longitude: {:.7f}".format(data.longitude))
    rospy.loginfo_throttle(5, "Altitude (WGS84): {:.2f} m".format(data.altitude))
    rospy.loginfo_throttle(5, "Covariancia de Posicao: {}".format(data.position_covariance))
    rospy.loginfo_throttle(5, "Tipo de Covariancia: {}".format(data.position_covariance_type))
    rospy.loginfo_throttle(5, "-----------------------------")

def gps_raw_listener():
    rospy.init_node('gps_raw_listener_node', anonymous=True)
    rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, gps_raw_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_raw_listener()
    except rospy.ROSInterruptException:
        pass