#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(data):
    rospy.loginfo_throttle(5, "----- Dados da IMU -----") # Loga a cada 5 segundos
    rospy.loginfo_throttle(5, "Orientacao (Quaternion):")
    rospy.loginfo_throttle(5, "  x: {:.4f}, y: {:.4f}, z: {:.4f}, w: {:.4f}".format(
        data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    rospy.loginfo_throttle(5, "Velocidade Angular (rad/s):")
    rospy.loginfo_throttle(5, "  x: {:.4f}, y: {:.4f}, z: {:.4f}".format(
        data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    rospy.loginfo_throttle(5, "Aceleracao Linear (m/s^2):")
    rospy.loginfo_throttle(5, "  x: {:.4f}, y: {:.4f}, z: {:.4f}".format(
        data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    rospy.loginfo_throttle(5, "-------------------------")

def imu_listener():
    rospy.init_node('imu_listener_node', anonymous=True)
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass