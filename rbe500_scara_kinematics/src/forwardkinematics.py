#!/usr/bin/env python
import math
from rbe500_scara_kinematics.srv import forwardkinService, forwardkinServiceResponse
import rospy


class FK:
    def __init__(self):
        self.server()

    def server(self):
        serviceFK = rospy.Service("ScaraFK", forwardkinService, self.doForwardKin)

    def doForwardKin(self, data):
        # Robot Parameters
        L1Vertical = 1.4
        L1Horizontal = 1.4
        L2 = 1.35
        L3 = 0.4

        q1 = data.q1
        q2 = data.q2
        q3 = data.q3
        print("Q Values")
        print(q1)
        print(q2)
        print(q3)

        # This code calculates each value of the FK
        # using equations found in matlab, also included in the submission
        row11 = math.cos(q1) * math.cos(q2) - math.sin(q1) * math.sin(q2)
        row12 = -math.cos(q1) * math.sin(q2) - math.cos(q2) * math.sin(q1)
        row13 = 0
        row14 = L1Horizontal*math.cos(q1)+L2*(math.cos(q1+q2)) #x

        row21 = math.cos(q1) * math.sin(q2) + math.cos(q2) * math.sin(q1)
        row22 = math.cos(q1) * math.cos(q2) - math.sin(q1) * math.sin(q2)
        row23 = 0
        row24 = L1Horizontal*math.sin(q1)+L2*(math.sin(q1+q2)) #y

        row31 = 0
        row32 = 0
        row33 = 1
        row34 = L1Vertical-L3-q3 #z

        row41 = 0
        row42 = 0
        row43 = 0
        row44 = 1

        x = row14
        y = row24
        z = row34

        psi = 0
        theta = 0
        phi = 0
        forceOrientation = 1

        # got from www.gregslabaugh.net/publications/euler.pdf
        if(row31 != 1 and row31 !=-1):
            theta = -math.asin(row31)
            if(forceOrientation):
                theta = -math.asin(row31)
            else:
                theta = math.pi-theta

            psi = math.atan2(row32 / math.cos(theta), row33 / math.cos(theta))
            phi = math.atan2(row21 / math.cos(theta), row11 / math.cos(theta))

        else:
            phi = 0
            if(row31 == -1):
                theta = math.pi/2
                psi = phi + math.atan2(row12, row13)
            else:
                theta = -math.pi/2
                psi = -phi + math.atan2(-row12, -row13)

        return forwardkinServiceResponse(x, y, z, phi, theta, psi)



if __name__ == '__main__':
    rospy.init_node('FK')
    forwardKin = FK()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        pass
