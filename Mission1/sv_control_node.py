#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import pigpio


pi = pigpio.pi()

def callback(data):
	direction = 1 - 2*data.buttons[0]

	dc_left = 300*direction*(-data.axes[4]+1) + 1500
	dc_right = 300*direction*(-data.axes[5]+1) + 1500

	sv_left = -900*data.axes[0] + 1500
	sv_right = -900*data.axes[2] + 1500
	
	print(dc_left, dc_right, sv_left, sv_right)
	pi.set_servo_pulsewidth(4,dc_left)
	pi.set_servo_pulsewidth(3,dc_right)
	pi.set_servo_pulsewidth(17,sv_left)
	pi.set_servo_pulsewidth(18,sv_right)	
	

# Intializes everything
def start():
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback)
	# starts the node
	rospy.init_node('sv_control_node')
	rospy.spin()

if __name__ == '__main__':
	start()

