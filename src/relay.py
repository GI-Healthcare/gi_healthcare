#!/usr/bin/env python


import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import String

import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# induction
LOCK = 26
ON = 6
HEAT_P = 19
HEAT_M = 13

# extraction fan
E_FAN = 20

# washing pump
PUMP = 16

# deliver actuator
WATER = 21

# deliver actuator
OIL = 12

# setup
GPIO.setup(LOCK,GPIO.OUT)
GPIO.setup(ON,GPIO.OUT)
GPIO.setup(HEAT_P,GPIO.OUT)
GPIO.setup(HEAT_M,GPIO.OUT)
GPIO.setup(E_FAN,GPIO.OUT)
GPIO.setup(PUMP,GPIO.OUT)
GPIO.setup(WATER,GPIO.OUT)
GPIO.setup(OIL,GPIO.OUT)

counter = 0
temp = 0

def callback(data):
    global counter
    d = data.data.split(",")
    deviceName = d[0]
    value = int(d[1])

    if deviceName == "I":
        if value >=-9 and value <=9:
            if value >0 and value <=9:
                temp = abs(value-counter)
                for i in range(temp):
                    GPIO.output(HEAT_P,GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(HEAT_P,GPIO.LOW)
                    time.sleep(0.2)
                    counter+=1
                
            if (value >=-9 and value <=0) and counter!=0:
                temp = counter-abs(value)
                for i in range(temp):
                    GPIO.output(HEAT_M,GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(HEAT_M,GPIO.LOW)
                    time.sleep(0.2)
                    counter-=1

        elif value == 10:
            GPIO.output(LOCK,GPIO.HIGH)
            time.sleep(2)
            GPIO.output(LOCK,GPIO.LOW)
        
        elif value == 11:
            GPIO.output(ON,GPIO.HIGH)
            time.sleep(2)
            GPIO.output(ON,GPIO.LOW)

        value = -5

    elif deviceName == "F":
        if value == 1:
            GPIO.output(E_FAN,GPIO.HIGH)
        else:
            GPIO.output(E_FAN,GPIO.LOW)
        value = -5

    elif deviceName == "P":
        if value == 1:
            GPIO.output(PUMP,GPIO.HIGH)
        else:
            GPIO.output(PUMP,GPIO.LOW)
        value = -5

    elif deviceName == "W":
        if value == 1:
            for i in range(2):
                GPIO.output(WATER,GPIO.HIGH)
        else:
            GPIO.output(WATER,GPIO.LOW)
        value = -5

    elif deviceName == "O":
        if value == 1:
            for i in range(2):
                GPIO.output(OIL,GPIO.HIGH)
        else:
            GPIO.output(OIL,GPIO.LOW)
        value = -5


def relaySubscriber():
	rospy.init_node('relayController')
	rospy.Subscriber('stepper_induction_command', String, callback)
	rospy.spin()	

if __name__ == '__main__':
	try:
		relaySubscriber()

	except rospy.ROSInterruptException:
		pass