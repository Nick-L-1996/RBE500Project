#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rbe500_scara_kinematics.srv import inverseVelkinService, inverseVelkinServiceResponse
from rbe500_scara_kinematics.srv import forwardVelkinService, forwardVelkinServiceResponse


class velocityKinematics:
    def __init__(self):
        self.jointSubscriber = rospy.Subscriber('scara/joint_states', JointState, self.updateRobotState, queue_size=1)
        self.Q1 = 0
        self.Q2 = 0
        self.Q3 = 0
        self.server()

    def server(self):
        serviceInverseKinVel = rospy.Service("ScaraIKVel", inverseVelkinService, self.inverseVelKin)
        serviceForwardKinVel = rospy.Service("ScaraFKVel", forwardVelkinService, self.forwardVelKin)

    def updateRobotState(self, data):
        self.Q1 = data.position[2]
        self.Q2 = data.position[1]
        self.Q3 = data.position[0]

    def inverseVelKin(self, data):
        linJac, angJac = self.CalcJacobian(self.Q1, self.Q2, self.Q3)
        linJacInv = np.linalg.inv(linJac)
        zeta = np.array[[data.x], [data.y], data.z]
        JointVel = linJacInv*zeta
        return inverseVelkinServiceResponse(JointVel[0], JointVel[1], JointVel[2])

    def forwardVelKin(self, data):
        linJac, angJac = self.CalcJacobian(self.Q1, self.Q2, self.Q3)
        qdot = np.array[[data.q1],[data.q2],data.q3]
        LinVel = linJac*qdot
        return forwardVelkinServiceResponse(LinVel[0], LinVel[1], LinVel[2])

    def CalcJacobian(self, q1, q2, q3):
        L1Vertical = 1.4
        L1Horizontal = 1
        L2 = 1.35
        L3 = 0.4
        item11 = -L2*np.sin(q1+q2)-L1Horizontal*np.sin(q1)
        item12 = -L2*np.sin(q1+q2)
        item13 = 0
        item21 = -L2 * np.cos(q1 + q2) - L1Horizontal * np.cos(q1)
        item22 = -L2 * np.cos(q1 + q2)
        item23 = 0
        item31 = 0
        item32 = 0
        item33 = 1
        JacobianLin = np.array([[item11, item12, item13], [item21, item22, item23],[item31, item32, item33]])

        item11 = 0
        item12 = 0
        item13 = 0
        item21 = 0
        item22 = 0
        item23 = 0
        item31 = 1
        item32 = 1
        item33 = 0
        JacobianAng = np.array([[item11, item12, item13], [item21, item22, item23],[item31, item32, item33]])

        return JacobianLin, JacobianAng

if __name__ == '__main__':
    rospy.init_node('VelocityKin')
    rospy.sleep(1)
    velKin = velocityKinematics()
    while not rospy.is_shutdown():
        pass