#!/usr/bin/env python
import rospy
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication
import sys

designerFile = "RBE500ProjP2UI.ui"
class P2Gui(QtWidgets.QMainWindow):
    def __init__(self):
        super(P2Gui, self).__init__()
        self.ui = uic.loadUi(designerFile, self)

if __name__ == '__main__':
    rospy.init_node('CommNode')
    rospy.sleep(1)
    app = QApplication(sys.argv)
    window = P2Gui()
    window.show()
    sys.exit(app.exec_())

