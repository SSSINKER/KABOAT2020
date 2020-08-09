#!/usr/bin/env python

import math
from math import sin, cos, pi
import serial
import rospy
import tf
import time
import re
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
    ser.write('<sor100>')
    print(ser.readline())
    #ser.write('<lpf20>')
    #print(ser.readline())

    imu_pub = rospy.Publisher("imu_data", Imu, queue_size=5)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
    odom_broadcaster = tf.TransformBroadcaster()
    stab_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10) # 10hz
    x = 0
    y = 0
    prev_str = ser.readline() # when prev_str is bad input -> :(
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        start_time = time.time()
        ser.reset_input_buffer()

        str_temp = ser.readline()
        comma_cnt = len([m.start() for m in re.finditer(',', str_temp)])
        if (comma_cnt != 9):
            str_list = prev_str
        else:
            str_list = str_temp
            prev_str = str_temp
        
        str_list = str_list.split(',')
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

        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        vx = float(str_list[7])
        vy = float(str_list[8])
        dx = vx * dt
        dy = vy * dt
        x += dx
        y += dy

        odom_quat = (float(str_list[2]),float(str_list[1]),float(str_list[0]),float(str_list[3]))
        euler = tf.transformations.euler_from_quaternion(odom_quat)
        # print(euler)

	# pitch?? ros ????? ??? ???? ?? ???? ????? ?? ??????? ?????
    # imu?? ?? ???? ?? ??? ? ? ??
        stab_quat = tf.transformations.quaternion_from_euler(euler[0],-euler[1],0)
        stab_broadcaster.sendTransform(
            (0, 0, 0),
            stab_quat,
            rospy.Time.now(),
            "base_link",
            "base_footprint"
        )

        odom_data.pose.pose.orientation.z = float(str_list[0])
        odom_data.pose.pose.orientation.y = float(str_list[1])
        odom_data.pose.pose.orientation.x = float(str_list[2])
        odom_data.pose.pose.orientation.w = float(str_list[3])
        odom_data.pose.pose.position.x = x
        odom_data.pose.pose.position.y = y
        odom_data.pose.pose.position.z = 0
        odom_data.twist.twist.linear.x = vx
        odom_data.twist.twist.linear.y = vy
        odom_data.twist.twist.linear.z = 0
        odom_data.twist.twist.angular.x = 0
        odom_data.twist.twist.angular.y = 0
        odom_data.twist.twist.angular.z = float(str_list[6])
        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.0),
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
        last_time = current_time

        prev_str = str_list
        end_time = time.time()
        print("time elapsed: {}".format(end_time - start_time))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass