#!/usr/bin/env python

import rospy
import os
from time import sleep
from std_msgs.msg import String
import xml.etree.ElementTree as ET

# ["S", "chopperLiftStepper", "gantryStepper", "utensilShaker"]
# ["I", " lock/unlock | on/off | +temp | -temp"]
#           10          11       [0:9]   [0:-9]

user = os.getlogin()

def recipeStepsRunner(recipeName):
    try:
        tree = ET.parse("/home/"+user+"/catkin_ws/src/gi_healthcare/recipes/recipeSteps.xml")
        
        root = tree.getroot()
        stepList = list()
        for recipe in root.findall("recipe"):
            if recipe.attrib["name"] == recipeName:
                for steps in recipe:
                    # stepper control
                    if steps.attrib["device"] == "S":
                        try:
                            stepList = [steps.attrib["device"],steps.attrib["stepper1_loc"],steps.attrib["stepper2_loc"]]
                            publishData(stepList)
                        except:
                            rospy.loginfo("Error in stepper tags in recipe xml file")

                    # induction control
                    elif steps.attrib["device"] == "I":
                        try:
                            stepList = [steps.attrib["device"], steps.attrib["value"]]
                            publishData(stepList)
                        except:
                            rospy.loginfo("Error in induction tags in recipe xml file")

                    # dispenser control
                    elif steps.attrib["device"] == "d":
                        try:
                            stepList = [steps.attrib["device"], steps.attrib["slot"]]
                            publishData(stepList)
                        except:
                            rospy.loginfo("Error in dispenser tags in recipe xml file")

                    # oil dispenser control
                    elif steps.attrib["device"] == "O":
                        try:
                            stepList = [steps.attrib["device"], steps.attrib["status"]]
                            publishData(stepList)
                        except:
                            rospy.loginfo("Error in oil tags in recipe xml file")

                    # water dispenser control
                    elif steps.attrib["device"] == "W":
                        try:
                            stepList = [steps.attrib["device"], steps.attrib["status"]]
                            publishData(stepList)
                        except:
                            rospy.loginfo("Error in water tags in recipe xml file")

                    # extraction fan control
                    elif steps.attrib["device"] == "F":
                        try:
                            stepList = [steps.attrib["device"], steps.attrib["status"]]
                            publishData(stepList)
                        except:
                            rospy.loginfo("Error in fan tags in wash xml file")

                    # delay control
                    elif steps.attrib["device"] == "D":
                        try:
                            stepDelay = [steps.attrib["device"], steps.attrib["seconds"]]
                            sleep(int(stepDelay[1]))
                        except:
                            rospy.loginfo("Error in delay tags in recipe xml file")

                    # default tag
                    else:
                        rospy.loginfo("Invalid tag in recipe xml file")
    except:
        rospy.loginfo("Unable to access recipe xml file")

def startWashing():
    try:
        tree = ET.parse("/home/"+user+"/catkin_ws/src/gi_healthcare/washingProcess/wash.xml")
        root = tree.getroot()
        stepList = list()
        for step in root.findall("wash"):
            # stepper control
            if step.attrib["device"] == "S":
                try:
                    stepList = [step.attrib["device"],step.attrib["stepper1_loc"],step.attrib["stepper2_loc"]]
                    publishData(stepList)
                except:
                    rospy.loginfo("Error in stepper tags in wash xml file")

            # washing pump control
            elif step.attrib["device"] == "P":
                try:
                    stepList = [step.attrib["device"], step.attrib["status"]]
                    publishData(stepList)
                except:
                    rospy.loginfo("Error in washing pump tags in wash xml file")

            # delay control
            elif step.attrib["device"] == "D":
                try:
                    stepDelay = [step.attrib["device"], step.attrib["seconds"]]
                    sleep(int(stepDelay[1]))
                except:
                    rospy.loginfo("Error in delay tags in wash xml file")
            
            # default
            else:
                rospy.loginfo("Invalid tag in wash xml file")
    except:
        rospy.loginfo("Unable to access wash xml file")

def shutdownProcess():
    stepperSteps = ["S","0000","0000","0000"]
    publishData(stepperSteps)
    sleep(3)
    
def publishData(d):
    d = ",".join(d)
    pub.publish(d)

def recipePublisher():
    flag = True
    while(flag):
        recipeStepsRunner("Noodles")
        flag = False
        

def callback(msg):
    # cooking cmd[0] = 1/0
    cmd = msg.data.split(",")
    if cmd[0] == "1":
        recipePublisher()   
    # washing cmd[1] = 0/1   
    elif  cmd[1] == "1":
        startWashing()
    # shutdown cmd[2] = 0/0   
    elif  cmd[0] == "0" and cmd[1] == "0":
        shutdownProcess()

if __name__ == '__main__':
    try:
        rospy.init_node('stepper_induction_node')
        rospy.Subscriber('control_command', String, callback)
        pub = rospy.Publisher('stepper_induction_command', String, queue_size = 1)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass