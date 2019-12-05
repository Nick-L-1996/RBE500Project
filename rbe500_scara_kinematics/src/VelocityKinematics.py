#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rbe500_scara_kinematics.srv import inverseVelkinService, inverseVelkinServiceResponse
from rbe500_scara_kinematics.srv import forwardVelkinService, forwardVelkinServiceResponse

#This node contains 2 services
#This first is the inverse velocity kinematics node
#This second is the forward kinematics node

class velocityKinematics:
    def __init__(self):
        self.server()

    def server(self):
        serviceInverseKinVel = rospy.Service("ScaraIKVel", inverseVelkinService, self.inverseVelKin)
        serviceForwardKinVel = rospy.Service("ScaraFKVel", forwardVelkinService, self.forwardVelKin)
        print("Services Started")

    def inverseVelKin(self, data):
        #Calculate Jacobian using sent joint angles q1, q2, q3
        linJac, angJac = self.CalcJacobian(data.q1, data.q2, data.q3)
        #calculate the inverse of the jacobian using numpy Library
        linJacInv = np.linalg.inv(linJac)
        #create zeta vector which is made of the linear velocities of x y and z
        zeta = np.array([[data.x], [data.y], [data.z]])
        #multiply inverse jacobian matrix by zeta vector (just like in the slides)
        JointVel = np.matmul(linJacInv, zeta)
        print("Did Inverse Kin")
        return inverseVelkinServiceResponse(float(JointVel[0]), float(JointVel[1]), float(JointVel[2]))


    def forwardVelKin(self, data):
        # Calculate Jacobian using sent joint angles q1, q2, q3
        linJac, angJac = self.CalcJacobian(data.q1, data.q2, data.q3)
        print("Linear Jacobian")
        print(linJac)
        #Create qdot vector using given angular velocities
        qdot = np.array([[data.q1dot],[data.q2dot],[data.q3dot]])
        print("QDot")
        print(qdot)
        #multiply jacobian by angular velocity vector
        LinVel = np.matmul(linJac, qdot)
        print(LinVel)
        print("Did forward Kin")
        return forwardVelkinServiceResponse(float(LinVel[0]), float(LinVel[1]), float(LinVel[2]))
        #return forwardVelkinServiceResponse(0,0,0)
    def CalcJacobian(self, q1, q2, q3):

        #creates the 3x3 jacobian matrix for linear velocities
        L1Vertical = 1.4
        L1Horizontal = 1.4
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

        #the 3x3 angular velocity components is the same for all configurations of this robot
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