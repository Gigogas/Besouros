#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu # O tópico data_raw também usa sensor_msgs/Imu
from functools import partial

IMU_RAW_TOPICS = [
    "/mavros/imu/data_raw",
    "/mavros/imu/data_raw2",
    "/mavros/imu/data_raw3"
]

def imu_raw_callback(data, topic_name):
    rospy.loginfo_throttle(5, "----- Dados Brutos da IMU de {} -----".format(topic_name))
    rospy.loginfo_throttle(5, "  Vel. Angular (rad/s): x: {:.3f}, y: {:.3f}, z: {:.3f}".format(
        data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    rospy.loginfo_throttle(5, "  Acel. Linear (m/s^2): x: {:.3f}, y: {:.3f}, z: {:.3f}".format(
        data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    rospy.loginfo_throttle(5, "-------------------------------------------")

def multi_imu_raw_listener():
    rospy.init_node('multi_imu_raw_listener_node', anonymous=True)
    rospy.loginfo("Procurando por topicos de IMU brutos...")

    available_topics = dict(rospy.get_published_topics())
    
    for topic in IMU_RAW_TOPICS:
        if topic in available_topics and available_topics[topic] == 'sensor_msgs/Imu':
            rospy.loginfo("Assinando o topico de IMU bruto: {}".format(topic))
            rospy.Subscriber(topic, Imu, partial(imu_raw_callback, topic_name=topic))

    rospy.loginfo("Escutando dados brutos das IMUs.")
    rospy.spin()

if __name__ == '__main__':
    multi_imu_raw_listener()