#!/usr/bin/env python

import rospy
from std_msgs.msg import String

# ["stepper", "washingPump"]

def publishData(data):
    data = ",".join(data)
    pub.publish(data)

def callback(msg):
    global data
    status = msg.data

    if status == "cooking":
        publishData(["1","0"])

    elif status == "washing":
        publishData(["0","1"])

    elif status == "shutdown":
        publishData(["0","0"])

if __name__ == '__main__':
    try:
        rospy.init_node('commander')
        rospy.Subscriber('ui_cmd', String, callback)
        pub = rospy.Publisher('control_command', String, queue_size = 1)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass