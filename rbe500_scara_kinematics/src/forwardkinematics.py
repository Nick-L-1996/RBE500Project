#!/usr/bin/env python
import math
from rbe500_scara_kinematics.msg import qVal
import rospy

class FK:
    def __init__(self):
        self._getQVals = rospy.Subscriber('/qVals', qVal, self.doForwardKin, queue_size=1)

    def doForwardKin(self, data):
        q1 = math.radians(data.q1)
        q2 = math.radians(data.q2)
        q3 = math.radians(data.q3)
        print("Output Matrix")
        print(math.cos(q2+q3)*math.cos(q1)),
        print(-math.sin(q2+q3)*math.cos(q1)),
        print(-math.sin(q1)),
        print(math.cos(q1)*math.cos(q2)-2*math.sin(q1)+math.cos(q1)*math.cos(q2)*math.cos(q3)-math.cos(q1)*math.sin(
            q2)*math.sin(q3))

        print(math.cos(q2 + q3) * math.sin(q1)),
        print(-math.sin(q2 + q3) * math.sin(q1)),
        print(-math.cos(q1)),
        print(math.sin(q1) * math.cos(q2) + 2 * math.cos(q1) + math.sin(q1) * math.sin(q2) * math.sin(q3) - math.cos(
            q3) * math.cos(q2) * math.sin(q1))
        print(-math.sin(q2+q3)),
        print(-math.cos(q2+q3)),
        print(0),
        print(1-math.sin(q2)-math.sin(q2+q3))

        print(0),
        print(0),
        print(0),
        print(1)



if __name__ == '__main__':
    rospy.init_node('FK')
    forwardKin = FK()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        pass
