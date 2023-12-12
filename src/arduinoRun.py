#!/usr/bin/env python

import serial
import time
import rospy
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

def callback(data):
    d = data.data
    ser.write(d)
    ser.reset_input_buffer()

def arduinoSubscriber():
	rospy.init_node('arduinoController')
	rospy.Subscriber('stepper_induction_command', String, callback)
	rospy.spin()	

if __name__ == '__main__':
	try:
		arduinoSubscriber()

	except rospy.ROSInterruptException:
		pass


