#!/usr/bin/env python
import math
from rbe500_scara_kinematics.srv import inversekinService, inversekinServiceResponse
from rbe500_scara_kinematics.srv import forwardkinService, forwardkinServiceResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import rospy


class part1ComNode:
    def __init__(self):
        serviceForwardKin = rospy.wait_for_service('ScaraFK')
        serviceInverseKin = rospy.wait_for_service('ScaraIK')
        self.useRviz = True#just use rviz for this assignment. Doesnt make sense to read joint states
        #from gazebo since we cant contol the robot yet
        if(self.useRviz):
            self.rvizJointSubscriber = rospy.Subscriber('/joint_states', JointState, self.updateRobotState, queue_size=1)
        else:
            self.jointSubscriber = rospy.Subscriber('scara/joint_states', JointState, self.updateRobotState, queue_size=1)
        self.currentRobotQ1 = 0
        self.currentRobotQ2 = 0
        self.currentRobotQ3 = 0

    def updateRobotState(self, data):
        #This constantly updates this nodes known position of the robot

        if(self.useRviz):
            self.currentRobotQ1 = data.position[0]
            self.currentRobotQ2 = data.position[1]
            self.currentRobotQ3 = data.position[2]
        else:
            self.currentRobotQ1 = data.position[2]
            self.currentRobotQ2 = data.position[1]
            self.currentRobotQ3 = data.position[0]

    #runs everytime the user presses a button
    def checkKinematics(self):
        #stores current robot position
        tempq1 = round(self.currentRobotQ1, 3)
        tempq2 = round(self.currentRobotQ2, 3)
        tempq3 = round(self.currentRobotQ3, 3)
        print("QValues from simulation")
        print("q1 = " + str(tempq1))
        print("q2 = " + str(tempq2))
        print("q3 = " + str(tempq3))

        #sends them to fk service
        sendFK = rospy.ServiceProxy('ScaraFK', forwardkinService)
        resp1 = sendFK(tempq1, tempq2, tempq3)
        #retrives fk from response
        x = resp1.x
        y = resp1.y
        z = resp1.z
        psi = resp1.psi
        theta = resp1.theta
        phi = resp1.phi
        print("Forward Kinematics Results")
        print("x = "+ str(x))
        print("y = " + str(y))
        print("z = " + str(z))
        print("psi = " + str(psi))
        print("theta = " + str(theta))
        print("phi = " + str(phi))

        #send fk to ik node in both elbow positions to have both possible positions known
        sendIK = rospy.ServiceProxy('ScaraIK', inversekinService)
        resp2 = sendIK(x, y, z, psi, theta, phi, 1)
        resp3 = sendIK(x, y, z, psi, theta, phi, 0)
        q1 = round(resp2.q1, 3)
        q2 = round(resp2.q2, 3)
        q3 = round(resp2.q3, 3)

        q11 = round(resp3.q1, 3)
        q21 = round(resp3.q2, 3)
        q31 = round(resp3.q3, 3)


        #checks to see if the result from the ik equals the original values when elbow = 1
        if(q1==tempq1 and q2 == tempq2 and q3==tempq3):
            print("Inverse Kinematics Results")
            print("Joint Positions match in the elbow left position")
            print("q1 = " + str(q1))
            print("q2 = " + str(q2))
            print("q3 = " + str(q3))
        #checks to see if the result from the ik equals the original values when elbow = 0
        elif (q11 == tempq1 and q21 == tempq2 and q31 == tempq3):
            print("Inverse Kinematics Results")
            print("Joint Positions match in the elbow right position")
            print("q1 = " + str(q11))
            print("q2 = " + str(q21))
            print("q3 = " + str(q31))
        else:
            print("Joint Positions Don't match")
            print("Elbow left")
            print("q1 = " + str(q1))
            print("q2 = " + str(q2))
            print("q3 = " + str(q3))
            print("Elbow right")
            print("q1 = " + str(q11))
            print("q2 = " + str(q21))
            print("q3 = " + str(q31))

if __name__ == '__main__':
    rospy.init_node('CommNode')
    com = part1ComNode()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        print("Type anything to test kinematics")
        text = raw_input()
        com.checkKinematics()


