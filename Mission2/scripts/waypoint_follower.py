#!/usr/bin/env python
import rospy
import math
import pigpio
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
import time

DIST_MAX = 4

pi = pigpio.pi()
pi2 = pigpio.pi()
pi2.set_PWM_frequency(17,50)

la_des = [37.455848, 37.3434343, 37.34343444, 37.43434343]
lo_des = [126.950599, 126.45555, 126.54444, 126.5455555]

yaw = 0
lo = 0
la = 0

def long_norm(longitude):
	longitude = 37 + (longitude - 3700) / 60
	return longitude

def lat_norm(latitude):
	latitude = 126 + (latitude - 12600) / 60 
	return latitude

def addAngle(angle1, angle2):
	result = angle1 + angle2
	if (result > 180):
		result -= 360
	elif (result < -180):
		result += 360
	return result

def getGPS(gps_data):
	global lo, la
	lo = gps_data.longitude
	la = gps_data.latitude
    
def getIMU(imu_data):
	global yaw
	orientation = imu_data.orientation
	quat = (orientation.x, orientation.y, orientation.z, orientation.w)
	euler = euler_from_quaternion(quat)
	yaw = euler[2] / math.pi * 180 # yaw in degree

def is_close(lo_diff, la_diff):
	global close_cnt
	a = lo_diff * 6380000 * math.pi / 180
	b = la_diff * 6380000 * math.pi / 180
	d = math.sqrt(a*a + b*b)
	if (d < DIST_MAX):
		return True
	else:
		return False


def waypoint_follower(lo_des, la_des):
	global lo, la, yaw

	n_waypoint = len(la_des)
	waypoint_idx = 0
	close_cnt = 0

	rospy.init_node('waypoint_follower', anonymous=True)

	dangle_pub = rospy.Publisher("dangle_deg", Float32)
	rate = rospy.Rate(10) # 10hz

	print("system preparing...")
	
	pi2.set_PWM_dutycycle(17,17)
	
	time.sleep(1)
	print("system starts!")

	while not rospy.is_shutdown():
		rospy.Subscriber("fix", NavSatFix, getGPS)
		rospy.Subscriber("imu_data", Imu , getIMU)

		lo_diff = lo_des[waypoint_idx] - lo
		la_diff = la_des[waypoint_idx] - la

		angle = math.acos(la_diff / math.sqrt(la_diff*la_diff + lo_diff*lo_diff)) / math.pi * 180 # degree

		if (lo_diff < 0):
			angle = -1 * angle

		# angle = math.atan(la_diff / lo_diff) / math.pi * 180

		dangle = addAngle(yaw, -angle)
		print(yaw)
		print(angle)

		dangle_pub.publish(dangle)

		if (dangle > 10):
			pi2.set_PWM_dutycycle(17,20)
			pi.set_servo_pulsewidth(18,1300)
			print("right")
		elif (dangle < -10):
			pi2.set_PWM_dutycycle(17,20)
			pi.set_servo_pulsewidth(18,1700)
	   		print("left")

		if (waypoint_idx < n_waypoint):
			if (is_close(lo_diff, la_diff)):
				close_cnt += 1
		# else:
			#stop ship

		if (close_cnt >= 5):
			close_cnt = 0
			if (waypoint_idx < n_waypoint - 1):
				waypoint_idx += 1

		rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_follower(lo_des, la_des)
    except rospy.ROSInterruptException:
        pass