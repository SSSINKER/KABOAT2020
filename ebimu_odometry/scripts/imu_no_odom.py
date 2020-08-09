#!/usr/bin/env python

import math
from math import sin, cos, pi
import serial
import rospy
import tf
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu


def talker():
    rospy.loginfo("Starting sensor publisher...")
    rospy.init_node('imu_no_odom_publisher')

    imu_port = "/dev/ttyUSB0"
    imu_data = Imu()
    odom_data = Odometry()
    ser = serial.Serial(imu_port, 115200) # determine manually
    ser.write('<sof2>')
    ser.readline()
    print(ser.readline())
    ser.write('<sog1>')
    print(ser.readline())
    ser.write('<soa5>')
    print(ser.readline())
    ser.write('<sem1>')
    print(ser.readline())
    ser.write('<sor10>')
    print(ser.readline())
    ser.write('<lpf20>')
    print(ser.readline())

    imu_pub = rospy.Publisher("imu/data", Imu, queue_size=1)
    stab_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(100) # 100hz
    x = 0
    y = 0
    while not rospy.is_shutdown():
        str_list = ser.readline().split(',')
        str_list[0] = str_list[0].split('*')[1]
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = "base_link"
        imu_data.orientation.z = float(str_list[0])
        imu_data.orientation.y = float(str_list[1])
        imu_data.orientation.x = float(str_list[2])
        imu_data.orientation.w = float(str_list[3])
        imu_data.linear_acceleration.x = 0
        imu_data.linear_acceleration.y = 0
        imu_data.linear_acceleration.z = 0
        imu_data.linear_acceleration_covariance[0] = -1
        imu_data.angular_velocity.x = 0
        imu_data.angular_velocity.y = 0
        imu_data.angular_velocity.z = 0
        imu_data.angular_velocity_covariance[0] = -1
        imu_pub.publish(imu_data)

        odom_quat = (float(str_list[2]),float(str_list[1]),float(str_list[0]),float(str_list[3]))
        euler = tf.transformations.euler_from_quaternion(odom_quat)
        # print(euler)
        stab_quat = tf.transformations.quaternion_from_euler(euler[0],-euler[1],0)

        stab_broadcaster.sendTransform(
            (0, 0, 0),
            stab_quat,
            rospy.Time.now(),
            "base_link",
            "base_footprint"
        )        

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
