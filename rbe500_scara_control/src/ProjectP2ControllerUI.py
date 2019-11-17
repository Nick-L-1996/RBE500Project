#!/usr/bin/env python
import rospy
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QSizePolicy
import sys
from std_msgs.msg import Float64

designerFile = "/media/nick/LinuxStorage/Documents/RBE500/catkin_ws/src/RBE500Project/rbe500_scara_control/src/RBE500ProjP2UI.ui"
class P2Gui(QtWidgets.QMainWindow):
    def __init__(self):
        super(P2Gui, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        self.J3Pub = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=1)
        self.ZPositionBTN.clicked.connect(self.sendZPos)

    def sendZPos(self):
        zPos = self.ZPosEntry.text()
        try:
            zPos = float(zPos)
        except Exception as e:
            self.ZPosEntry.setText("0")
            zPos = 0.0
        self.J3Pub.publish(zPos)






if __name__ == '__main__':
    rospy.init_node('Part2Controller')
    rospy.sleep(1)
    app = QApplication(sys.argv)
    window = P2Gui()
    window.show()
    sys.exit(app.exec_())

