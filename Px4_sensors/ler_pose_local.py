#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

def local_pos_callback(data):
    rospy.loginfo_throttle(5, "----- Posicao Local Estimada (ENU) -----") # Ângulos em quaternion (East-North-Up)
    rospy.loginfo_throttle(5, "Timestamp: {}.{}".format(data.header.stamp.secs, data.header.stamp.nsecs))
    rospy.loginfo_throttle(5, "Posicao (m):")
    rospy.loginfo_throttle(5, "  x: {:.3f}, y: {:.3f}, z: {:.3f}".format(
        data.pose.position.x, data.pose.position.y, data.pose.position.z))
    rospy.loginfo_throttle(5, "Orientacao (Quaternion):")
    rospy.loginfo_throttle(5, "  x: {:.3f}, y: {:.3f}, z: {:.3f}, w: {:.3f}".format(
        data.pose.orientation.x, data.pose.orientation.y,
        data.pose.orientation.z, data.pose.orientation.w))
    
    rospy.loginfo_throttle(5, "----- Roll, Pitch e Yaw -----") # Ângulos em euler 
    orientation_q = data.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    rospy.loginfo_throttle(5, "Orientacao (Euler, rad): Roll: {:.3f}, Pitch: {:.3f}, Yaw: {:.3f}".format(roll, pitch, yaw))
    
    rospy.loginfo_throttle(5, "--------------------------------------")

def local_pos_listener():
    rospy.init_node('local_pos_listener_node', anonymous=True)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_pos_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        local_pos_listener()
    except rospy.ROSInterruptException:
        pass