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
from datetime import datetime
import numpy as np
designerFile = "/media/nick/LinuxStorage/Documents/RBE500/catkin_ws/src/RBE500Project/rbe500_scara_control/src/RBE500ProjP3UI.ui"
class P3Gui(QtWidgets.QMainWindow):
    def __init__(self):
        super(P3Gui, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        self.runPlot = False
        self.J1PosPub = rospy.Publisher('/scara/joint1_position_controller/command', Float64, queue_size=1)
        self.J2PosPub = rospy.Publisher('/scara/joint2_position_controller/command', Float64, queue_size=1)
        self.J3PosPub = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=1)
        self.jointSubscriber = rospy.Subscriber('scara/joint_states', JointState, self.updateRobotState, queue_size=1)
        serviceInverseKin = rospy.wait_for_service('ScaraIK')
        serviceForwardKin = rospy.wait_for_service('ScaraFK')
        serviceInverseKinVel = rospy.wait_for_service('ScaraIKVel')
        serviceForwardKinVel = rospy.wait_for_service('ScaraFKVel')
        self.currentRobotQ1 = 0
        self.currentRobotQ2 = 0
        self.currentRobotQ3 = 0
        self.times = []
        self.positionsx = []
        self.positionsy = []
        self.positionsz = []
        self.startTime = datetime.now()
        self.MoveBTN.clicked.connect(self.sendZPos)
        self.StopPlotting.clicked.connect(self.stopPlot)
        self.XPlot = PlotCanvas(parent=self.frameXPos)
        self.YPlot = PlotCanvas(parent=self.frameYPos)
        self.ZPlot = PlotCanvas(parent=self.frameZPos)
        self.DesiredXPos = 0
        self.DesiredYPos = 0
        self.DesiredZPos = 0


    def sendZPos(self):
        xPos = self.XPosEntry.text()
        yPos = self.YPosEntry.text()
        zPos = self.ZPosEntry.text()
        try:
            xPos = float(xPos)
        except Exception as e:
            self.XPosEntry.setText("0")
            xPos = 0.0
        try:
            yPos = float(yPos)
        except Exception as e:
            self.YPosEntry.setText("0")
            yPos = 0.0
        try:
            zPos = float(zPos)
        except Exception as e:
            self.ZPosEntry.setText("0")
            zPos = 0.0
        sendIK = rospy.ServiceProxy('ScaraIK', inversekinService)
        resp1 = sendIK(xPos, yPos, zPos, 0, 0, 0, 1)
        self.times = []
        self.positionsx = []
        self.positionsy = []
        self.positionsz = []
        self.startTime = datetime.now()
        self.DesiredXPos = xPos
        self.DesiredYPos = yPos
        self.DesiredZPos = zPos
        self.J1PosPub.publish(resp1.q1)
        self.J2PosPub.publish(resp1.q2)
        self.J3PosPub.publish(resp1.q3)
        self.XPlot.plotPos(self.times, self.positionsx, self.DesiredXPos, "X")
        self.YPlot.plotPos(self.times, self.positionsy, self.DesiredYPos, "Y")
        self.ZPlot.plotPos(self.times, self.positionsz, self.DesiredZPos, "Z")
        self.runPlot = True

    def updateRobotState(self, data):
        self.currentRobotQ1 = data.position[2]
        self.currentRobotQ2 = data.position[1]
        self.currentRobotQ3 = data.position[0]
        sendFK = rospy.ServiceProxy('ScaraFK', forwardkinService)
        resp1 = sendFK(self.currentRobotQ1, self.currentRobotQ2, self.currentRobotQ3)
        self.CurrentXPosLBL.setText("Current X Position: " + str(round(resp1.x, 2)) + "m")
        self.CurrentYPosLBL.setText("Current Y Position: " + str(round(resp1.y, 2)) + "m")
        self.CurrentZPosLBL.setText("Current Z Position: " + str(round(resp1.z, 2)) + "m")
        if self.runPlot:
            timenow = datetime.now()-self.startTime
            timenow = timenow.total_seconds()
            self.times.append(float(timenow))
            self.positionsx.append(resp1.x)
            self.positionsy.append(resp1.y)
            self.positionsz.append(resp1.z)
            self.XPlot.plotPos(self.times, self.positionsx, self.DesiredXPos, "X")
            self.YPlot.plotPos(self.times, self.positionsy, self.DesiredYPos, "Y")
            self.ZPlot.plotPos(self.times, self.positionsz, self.DesiredZPos, "Z")

    def stopPlot(self):
        self.runPlot=False

class PlotCanvas(FigureCanvas):

    def __init__(self, parent=None, width=4, height=2.5, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QSizePolicy.Expanding,
                                   QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


    def plotPos(self, time, position, desired, axis):
        self.axes.axes.cla()
        line = desired*np.ones(len(time))
        self.axes.plot(time, line, 'r-', label="Desired Position")
        self.axes.plot(time, position, 'b-', label="Actual Position")
        self.axes.set_xlabel("Time (s)")
        self.axes.set_ylabel(axis+" Position (meters)")
        self.axes.set_title("Plot of "+axis+" Position")
        self.draw()

    def plotVel(self, time, velocity, desired, axis):
        self.axes.axes.cla()
        line = desired * np.ones(len(time))
        self.axes.plot(time, line, 'r-', label="Desired Velocity")
        self.axes.plot(time, velocity, 'b-', label="Actual Velocity")
        self.axes.set_xlabel("Time (s)")
        self.axes.set_ylabel(axis + " Velocity (meters/sec)")
        self.axes.set_title("Plot of " + axis + " Velocity")
        self.draw()


if __name__ == '__main__':
    rospy.init_node('Part2Controller')
    rospy.sleep(1)
    app = QApplication(sys.argv)
    window = P3Gui()
    window.show()
    sys.exit(app.exec_())

