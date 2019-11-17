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
        L1Vertical = 0.2
        L1Horizontal = 0.2
        L2 = 0.2
        L3=0.1
        q2 = 0

        try:
            q2 = math.acos(math.pow(data.x, 2)+math.pow(data.y, 2)
                           - (1*math.pow(L1Horizontal, 2)-math.pow(L2, 2))/(2*L1Horizontal*L2))

        except ValueError as e:
            q2 = 0

        if data.elbow:
            q2 = -q2
        q1 = math.atan2(data.y, data.x)-math.atan2(L2*math.sin(q2), L1Horizontal+L2*math.cos(q2))
        q3 = data.z-L1Vertical+L3

        return inversekinServiceResponse(q1, q2, q3)


if __name__ == '__main__':
    rospy.init_node('FK')
    forwardKin = inverseKin()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        pass
