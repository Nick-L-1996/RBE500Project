#!/usr/bin/env python
import rospy
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QSizePolicy
import sys
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rbe500_scara_kinematics.srv import inversekinService, inversekinServiceResponse
from rbe500_scara_kinematics.srv import forwardkinService, forwardkinServiceResponse
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib
from PyQt5.QtCore import QThread, pyqtSignal
from datetime import datetime
import numpy as np
designerFile = "/media/nick/LinuxStorage/Documents/RBE500/catkin_ws/src/RBE500Project/rbe500_scara_control/src/RBE500ProjP2UI.ui"
class P2Gui(QtWidgets.QMainWindow):
    def __init__(self):
        super(P2Gui, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        self.J3Pub = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=1)
        self.jointSubscriber = rospy.Subscriber('scara/joint_states', JointState, self.updateRobotState, queue_size=1)
        serviceInverseKin = rospy.wait_for_service('ScaraIK')
        serviceForwardKin = rospy.wait_for_service('ScaraFK')
        self.runPlot = False
        self.currentRobotQ1 = 0
        self.currentRobotQ2 = 0
        self.currentRobotQ3 = 0
        self.times = []
        self.positions = []
        self.startTime = datetime.now()
        self.ZPositionBTN.clicked.connect(self.sendZPos)
        self.StopPlotting.clicked.connect(self.stopPlot)
        self.ZPlot = PlotCanvas(parent=self.frame)
        self.Desired = 0


    def sendZPos(self):
        zPos = self.ZPosEntry.text()
        try:
            zPos = float(zPos)
        except Exception as e:
            self.ZPosEntry.setText("0")
            zPos = 0.0
        sendIK = rospy.ServiceProxy('ScaraIK', inversekinService)
        resp1 = sendIK(0, 0, zPos, 0, 0, 0, 1)
        self.times = []
        self.positions = []
        self.startTime = datetime.now()
        self.Desired = zPos
        self.J3Pub.publish(resp1.q3)
        self.ZPlot.plot(self.times, self.positions, self.Desired)
        self.runPlot=True

    def updateRobotState(self, data):
        self.currentRobotQ1 = data.position[2]
        self.currentRobotQ2 = data.position[1]
        self.currentRobotQ3 = data.position[0]
        sendFK = rospy.ServiceProxy('ScaraFK', forwardkinService)
        resp1 = sendFK(0, 0, self.currentRobotQ3)
        self.label_4.setText("Current Z Position: "+ str(round(resp1.z, 2))+"m")
        if(self.runPlot):
            timenow = datetime.now()-self.startTime
            timenow = timenow.total_seconds()
            self.times.append(float(timenow))
            self.positions.append(resp1.z)
            self.ZPlot.plot(self.times, self.positions, self.Desired)

    def stopPlot(self):
        self.runPlot=False

class PlotCanvas(FigureCanvas):

    def __init__(self, parent=None, width=6, height=4.8, dpi=95):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QSizePolicy.Expanding,
                                   QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


    def plot(self, time, position, desired):
        self.axes.axes.cla()
        line = desired*np.ones(len(time))
        self.axes.plot(time, line, 'r-', label="Desired Position")
        self.axes.plot(time, position, 'b-', label="Actual Position")
        self.axes.set_xlabel("Data Points")
        self.axes.set_ylabel("Confidence")
        self.axes.set_title("Confidence of Prediction in Real Time")
        self.draw()








if __name__ == '__main__':
    rospy.init_node('Part2Controller')
    rospy.sleep(1)
    app = QApplication(sys.argv)
    window = P2Gui()
    window.show()
    sys.exit(app.exec_())

