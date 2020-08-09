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
    rospy.init_node('imu_odom_publisher')

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
    ser.write('<sod1>')
    print(ser.readline())
    ser.write('<sor10>')
    print(ser.readline())
    ser.write('<lpf20>')
    print(ser.readline())

    imu_pub = rospy.Publisher("imu/data", Imu, queue_size=1)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(100) # 100hz


    while not rospy.is_shutdown():
        str_list = ser.readline().split(',')
        str_list[0] = str_list[0].split('*')[1]
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = "base_link"
        imu_data.orientation.x = float(str_list[2])
        imu_data.orientation.y = float(str_list[1])
        imu_data.orientation.z = float(str_list[0])
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
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, euler[2])
        odom_data.pose.pose.orientation.x = float(str_list[2])
        odom_data.pose.pose.orientation.y = float(str_list[1])
        odom_data.pose.pose.orientation.z = float(str_list[0])
        odom_data.pose.pose.orientation.w = float(str_list[3])
        odom_data.pose.pose.position.x = float(str_list[10])
        odom_data.pose.pose.position.y = float(str_list[11])
        odom_data.pose.pose.position.z = 0
        odom_data.twist.twist.linear.x = float(str_list[7])
        odom_data.twist.twist.linear.y = float(str_list[8])
        odom_data.twist.twist.linear.z = 0
        odom_data.twist.twist.angular.x = 0
        odom_data.twist.twist.angular.y = 0
        odom_data.twist.twist.angular.z = float(str_list[6])
        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (float(str_list[10]), float(str_list[11]), 0.0),
            odom_quat,
            rospy.Time.now(),
            "base_footprint",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom_data.header.stamp = rospy.Time.now()
        odom_data.header.frame_id = "odom"

        # set the velocity
        odom_data.child_frame_id = "base_footprint"

        # publish the message
        odom_pub.publish(odom_data)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
