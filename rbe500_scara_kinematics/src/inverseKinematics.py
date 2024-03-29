#!/usr/bin/env python
import math
from rbe500_scara_kinematics.srv import inversekinService, inversekinServiceResponse
import rospy


class inverseKin:
    def __init__(self):
        self.server()

    def server(self):
        serviceInverseKin = rospy.Service("ScaraIK", inversekinService, self.inverseKin)

    def inverseKin(self, data):
        #Robot Parameters
        L1Vertical = 1.4
        L1Horizontal = 1.4
        L2 = 1.35
        L3=0.4
        q2 = 0
        print("Positions")
        print(data.x)
        print(data.y)
        print(data.z)

        #This is in a try except statement just incase there specified position is unreachable and acos returns an error
        try:
            q2 = math.acos((math.pow(data.x, 2)+math.pow(data.y, 2)
                           - (math.pow(L1Horizontal, 2)+math.pow(L2, 2)))/(2*L1Horizontal*L2))
        except ValueError as e:
        #if there is an error force q2 to 0
            q2 = 0

        #specifies what elbow position the robot should be in
        if data.elbow==1:
            q2 = -q2

        q1 = math.atan2(data.y, data.x)-math.atan2(L2*math.sin(q2), L1Horizontal+L2*math.cos(q2))
        if q1 < 0:
            q1 += 2*math.pi
        q3 = L1Vertical-L3-data.z


        return inversekinServiceResponse(q1, q2, q3)


if __name__ == '__main__':
    rospy.init_node('IK')
    forwardKin = inverseKin()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        pass
