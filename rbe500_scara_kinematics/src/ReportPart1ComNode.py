#!/usr/bin/env python
import math
from rbe500_scara_kinematics.srv import inversekinService, inversekinServiceResponse
from rbe500_scara_kinematics.srv import forwardkinService, forwardkinServiceResponse
import rospy


class part1ComNode:
    def __init__(self):
        serviceForwardKin = rospy.wait_for_service('ScaraFK')
        serviceInverseKin = rospy.wait_for_service('ScaraIK')
        self.jointSubscriber = rospy.Subscriber('gazebo/link_states', self.updateRobotState)

    def updateRobotState(self, data):


