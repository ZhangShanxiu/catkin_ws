#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from std_msgs.msg import Float64
from dynamixel_controllers.srv import TorqueEnable, TorqueEnableRequest
from dynamixel_controllers.srv import SetSpeed, SetSpeedRequest, SetSpeedResponse
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class DoubleEnableOrNot:
    def __init__(self):

        if len(sys.argv) != 2:
            print('Usage: %s x' %sys.argv[0])
            sys.exit(1)

        opt = int(sys.argv[1])

        if opt:
            # 使能
            try:
                cute1_go_home = rospy.ServiceProxy('/cute_robot_1/cute_go_home', SetBool)
                result1 = cute1_go_home(True)
                print('Result1: %s' %result1)
                rospy.sleep(2)
                cute2_go_home = rospy.ServiceProxy('/cute_robot_2/cute_go_home', SetBool)
                result1 = cute2_go_home(True)
                print('Result2: %s' %result1)
            except rospy.ServiceException, e:
                #print "Service call failed: %s"%e
                print('Service /cute_go_home call failed: %s' %e)
        else:
            # 失能
            try:
                cute1_disable = rospy.ServiceProxy('/cute_robot_1/cute_torque_enable', SetBool)
                result1 = cute1_disable(False)
                print('Result1: %s' %result1)
                rospy.sleep(2)
                cute2_disable = rospy.ServiceProxy('/cute_robot_2/cute_torque_enable', SetBool)
                result1 = cute2_disable(False)
                print('Result2: %s' %result1)
            except rospy.ServiceException, e:
                #print "Service call failed: %s"%e
                print('Service /cute_go_home call failed: %s' %e)

if __name__ == "__main__":
    DoubleEnableOrNot()