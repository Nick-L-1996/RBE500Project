#!/usr/bin/env python
import rospy
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QSizePolicy
import sys
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rbe500_scara_kinematics.srv import inversekinService, inversekinServiceResponse
from rbe500_scara_kinematics.srv import forwardkinService, forwardkinServiceResponse

designerFile = "/media/nick/LinuxStorage/Documents/RBE500/catkin_ws/src/RBE500Project/rbe500_scara_control/src/RBE500ProjP2UI.ui"
class P2Gui(QtWidgets.QMainWindow):
    def __init__(self):
        super(P2Gui, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        self.J3Pub = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=1)
        self.jointSubscriber = rospy.Subscriber('scara/joint_states', JointState, self.updateRobotState, queue_size=1)
        serviceInverseKin = rospy.wait_for_service('ScaraIK')
        serviceForwardKin = rospy.wait_for_service('ScaraFK')
        self.currentRobotQ1 = 0
        self.currentRobotQ2 = 0
        self.currentRobotQ3 = 0
        self.ZPositionBTN.clicked.connect(self.sendZPos)


    def sendZPos(self):
        zPos = self.ZPosEntry.text()
        try:
            zPos = float(zPos)
        except Exception as e:
            self.ZPosEntry.setText("0")
            zPos = 0.0
        sendIK = rospy.ServiceProxy('ScaraIK', inversekinService)
        resp1 = sendIK(0, 0, zPos, 0, 0, 0, 1)
        print("sending q3")
        print(resp1.q3)
        self.J3Pub.publish(resp1.q3)

    def updateRobotState(self, data):
        self.currentRobotQ1 = data.position[2]
        self.currentRobotQ2 = data.position[1]
        self.currentRobotQ3 = data.position[0]
        sendFK = rospy.ServiceProxy('ScaraFK', forwardkinService)
        resp1 = sendFK(0, 0, self.currentRobotQ3)
        self.label_4.setText("Current Z Position: "+ str(round(resp1.z, 2))+"m")






if __name__ == '__main__':
    rospy.init_node('Part2Controller')
    rospy.sleep(1)
    app = QApplication(sys.argv)
    window = P2Gui()
    window.show()
    sys.exit(app.exec_())

