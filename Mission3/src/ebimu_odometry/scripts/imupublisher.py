#!/usr/bin/env python

import serial
import rospy
import tf
import time

from sensor_msgs.msg import Imu

def talker():
    rospy.loginfo("Starting sensor publisher...")
    rospy.init_node('imu_publisher')
    imu_port = "/dev/ttyUSB1"
    imu_data = Imu()
    ser = serial.Serial(imu_port, 115200) # determine manually
    ser.write('<sof2>')
    ser.readline()
    print(ser.readline())
    ser.write('<sog1>')
    print(ser.readline())
    ser.write('<soa1>')
    print(ser.readline())
    ser.write('<sem1>')
    print(ser.readline())
    ser.write('<sor10>')

    sensor_pub = rospy.Publisher("imu/data", Imu, queue_size=1)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        str_list = ser.readline().rstrip().split(',')
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = "base_link"
        # imu_data.header.seq = seq
        str_list[0] = str_list[0].split('*')[1]
        imu_data.orientation.w = float(str_list[3])
        imu_data.orientation.x = float(str_list[2])
        imu_data.orientation.y = float(str_list[1])
        imu_data.orientation.z = float(str_list[0])
        imu_data.linear_acceleration.x = float(str_list[7])
        imu_data.linear_acceleration.y = float(str_list[8])
        imu_data.linear_acceleration.z = float(str_list[9])
        imu_data.linear_acceleration_covariance[0] = -1
        imu_data.angular_velocity.x = float(str_list[4])
        imu_data.angular_velocity.y = float(str_list[5])
        imu_data.angular_velocity.z = float(str_list[6])
        imu_data.angular_velocity_covariance[0] = -1
        rospy.loginfo(imu_data)
        sensor_pub.publish(imu_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
