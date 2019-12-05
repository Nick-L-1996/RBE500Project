#!/usr/bin/env python
import rospy
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QSizePolicy
from PyQt5.QtCore import QThread, pyqtSignal
import sys
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rbe500_scara_kinematics.srv import inversekinService, inversekinServiceResponse
from rbe500_scara_kinematics.srv import forwardkinService, forwardkinServiceResponse
from rbe500_scara_kinematics.srv import inverseVelkinService, inverseVelkinServiceResponse
from rbe500_scara_kinematics.srv import forwardVelkinService, forwardVelkinServiceResponse
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
        #trajectory controller publisher
        self.TrajControl = rospy.Publisher('/scara/traj_controller/command', JointTrajectory,  queue_size=1)
        self.jointSubscriber = rospy.Subscriber('scara/joint_states', JointState, self.updateRobotState, queue_size=100)
        serviceInverseKin = rospy.wait_for_service('ScaraIK')
        serviceForwardKin = rospy.wait_for_service('ScaraFK')
        serviceInverseKinVel = rospy.wait_for_service('ScaraIKVel')
        serviceForwardKinVel = rospy.wait_for_service('ScaraFKVel')

        #variables for keeping track of robot state and past states for plotting
        self.currentRobotQ1 = 0
        self.currentRobotQ2 = 0
        self.currentRobotQ3 = 0
        self.currentRobotQ1Dot = 0
        self.currentRobotQ2Dot = 0
        self.currentRobotQ3Dot = 0
        self.times = []
        self.positionsq1 = []
        self.positionsq2 = []
        self.positionsq3 = []
        self.velocitiesq1 = []
        self.velocitiesq2 = []
        self.velocitiesq3 = []
        self.positionsx = []
        self.positionsy = []
        self.positionsz = []
        self.velocitiesx = []
        self.velocitiesy = []
        self.velocitiesz = []
        self.startTime = datetime.now()

        #attaches functions to buttons
        self.MoveBTN.clicked.connect(self.sendTraj)
        self.StopPlotting.clicked.connect(self.stopPlot)
        #Create all plots
        #position plots
        self.XPlot = PlotCanvas(parent=self.frameXPos)
        self.YPlot = PlotCanvas(parent=self.frameYPos)
        self.ZPlot = PlotCanvas(parent=self.frameZPos)
        #velocity plots
        self.XVelPlot = PlotCanvas(parent=self.frameXVel)
        self.YVelPlot = PlotCanvas(parent=self.frameYVel)
        self.ZVelPlot = PlotCanvas(parent=self.frameZVel)
        #initial values for desired state
        self.DesiredXPos = 0
        self.DesiredYPos = 0
        self.DesiredZPos = 0
        self.DesiredXVel = 0
        self.DesiredYVel = 0
        self.DesiredZVel = 0
        #the trajectory is calculated in a seperate thread to prevent
        #the gui from becoming unresponsive
        self.trajCalcThread = CalculateTrajectoryThread(self)
        self.trajCalcThread.signal.connect(self.plotDataStart)


    def sendTraj(self):
        self.trajCalcThread.start()


    def updateRobotState(self, data):
        #when a new robot state is sent grom gazebo
        #This function updates the stored state of the robot
        #and updates the labels in the gui
        self.currentRobotQ1 = round(data.position[2], 2)
        self.currentRobotQ2 = round(data.position[1], 2)
        self.currentRobotQ3 = round(data.position[0], 2)
        self.currentRobotQ1Dot = round(data.velocity[2], 2)
        self.currentRobotQ2Dot = round(data.velocity[1], 2)
        self.currentRobotQ3Dot = round(data.velocity[0], 2)
        #finds forward kinematics for plotting and labels
        sendFK = rospy.ServiceProxy('ScaraFK', forwardkinService)
        resp1 = sendFK(self.currentRobotQ1, self.currentRobotQ2, self.currentRobotQ3)
        self.CurrentXPosLBL.setText("Current X Position: " + str(round(resp1.x, 2)) + "m")
        self.CurrentYPosLBL.setText("Current Y Position: " + str(round(resp1.y, 2)) + "m")
        self.CurrentZPosLBL.setText("Current Z Position: " + str(round(resp1.z, 2)) + "m")

        if self.runPlot:
            #if the run plot is active it will store the positions and velocities of each joint
            timenow = datetime.now()-self.startTime
            timenow = timenow.total_seconds()
            self.times.append(float(timenow))
            self.positionsx.append(resp1.x)
            self.positionsy.append(resp1.y)
            self.positionsz.append(resp1.z)
            self.positionsq1.append(self.currentRobotQ1)
            self.positionsq2.append(self.currentRobotQ2)
            self.positionsq3.append(self.currentRobotQ3)
            self.velocitiesq1.append(self.currentRobotQ1Dot)
            self.velocitiesq2.append(self.currentRobotQ2Dot)
            self.velocitiesq3.append(self.currentRobotQ3Dot)


    def stopPlot(self):
        #when the stop button is pressed data stops being collected
        #and it will generate all of the plots
        #doing forward velocitiy kinematics as needed
        if(self.runPlot):
            self.runPlot = False
            self.XPlot.plotPos(self.times, self.positionsx, self.DesiredXPos, "X")
            self.YPlot.plotPos(self.times, self.positionsy, self.DesiredYPos, "Y")
            self.ZPlot.plotPos(self.times, self.positionsz, self.DesiredZPos, "Z")

            sendFKVel = rospy.ServiceProxy('ScaraFKVel', forwardVelkinService)

            for i in range(0, len(self.times)):
                resp2 = sendFKVel(self.positionsq1[i], self.positionsq2[i], self.positionsq3[i], self.velocitiesq1[i],
                                  self.velocitiesq2[i], self.velocitiesq3[i])
                self.velocitiesx.append(-resp2.x)
                self.velocitiesy.append(-resp2.y)
                self.velocitiesz.append(-resp2.z)

            self.XVelPlot.plotVel(self.times, self.velocitiesx, self.DesiredXVel, "X")
            self.YVelPlot.plotVel(self.times, self.velocitiesy, self.DesiredYVel, "Y")
            self.ZVelPlot.plotVel(self.times, self.velocitiesz, self.DesiredZVel, "Z")

    def plotDataStart(self, results):
        #this is used as a software interrupt callback from the
        #trajectory thread. This starts plotting before the robot is told
        #to move
        self.runPlot = True
        print("Refreshing Plots")

class PlotCanvas(FigureCanvas):

    #class for the graphs
    def __init__(self, parent=None, width=6, height=4, dpi=100):
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

#This calculates the trajectory for the robot in a seperate thread
class CalculateTrajectoryThread(QThread):
    signal = pyqtSignal('PyQt_PyObject')

    def __init__(self, gui):
        QThread.__init__(self)
        self.gui = gui

    def run(self):
        xPos = self.gui.XPosEntry.text()
        yPos = self.gui.YPosEntry.text()
        zPos = self.gui.ZPosEntry.text()
        xVel = self.gui.XVelEntry.text()
        yVel = self.gui.YVelEntry.text()
        zVel = self.gui.ZVelEntry.text()
        #verifies all text entries are numbers
        try:
            xPos = float(xPos)
        except Exception as e:
            #self.gui.XPosEntry.setText("0")
            xPos = 0.0
        try:
            yPos = float(yPos)
        except Exception as e:
            #self.gui.YPosEntry.setText("0")
            yPos = 0.0
        try:
            zPos = float(zPos)
        except Exception as e:
            #self.gui.ZPosEntry.setText("0")
            zPos = 0.0
        try:
            xVel = float(xVel)
        except Exception as e:
            #self.gui.XVelEntry.setText("0.1")
            xVel = 0.1
        try:
            yVel = float(yVel)
        except Exception as e:
            #self.gui.YVelEntry.setText("0.1")
            yVel = 0.1
        try:
            zVel = float(zVel)
        except Exception as e:
            #self.gui.ZVelEntry.setText("0.1")
            zVel = 0.1
        self.gui.startTime = datetime.now()
        self.gui.DesiredXPos = xPos
        self.gui.DesiredYPos = yPos
        self.gui.DesiredZPos = zPos
        self.gui.DesiredXVel = xVel
        self.gui.DesiredYVel = yVel
        self.gui.DesiredZVel = zVel


        sendFK = rospy.ServiceProxy('ScaraFK', forwardkinService)
        resp1 = sendFK(self.gui.currentRobotQ1, self.gui.currentRobotQ2, self.gui.currentRobotQ3)

        #Start creating trajectory message
        trajToBeSent = JointTrajectory()
        trajToBeSent.joint_names.append("baseToJOne")
        trajToBeSent.joint_names.append("JTwoToJThree")
        trajToBeSent.joint_names.append("JFiveToJSix")

        #verifies the 2 positions dont result in a
        #straight line trajectory through the unreachable
        #areas of the robot
        if self.verifyTraj(resp1.x, resp1.y, resp1.z, xPos, yPos, zPos):
            #if the trajectory is possible
            #Do Linear Trajectory
            sendFKVel = rospy.ServiceProxy('ScaraFKVel', forwardVelkinService)
            resp2 = sendFKVel(self.gui.currentRobotQ1, self.gui.currentRobotQ2, self.gui.currentRobotQ3, self.gui.currentRobotQ1Dot, self.gui.currentRobotQ2Dot, self.gui.currentRobotQ3Dot)
            trajToBeSent.points = self.createTraj(resp1.x, resp1.y, resp1.z, xPos, yPos, zPos,resp2.x, resp2.y, resp2.z, xVel, yVel, zVel)
            print(trajToBeSent)
        else:
            #if it is not possible
            #do Setpoint movement
            sendIK = rospy.ServiceProxy('ScaraIK', inversekinService)
            resp2 = sendIK(xPos, yPos, zPos, 0, 0, 0, 1)
            setPoint = JointTrajectoryPoint()
            setPoint.positions.append(float(resp2.q1))
            setPoint.positions.append(float(resp2.q2))
            setPoint.positions.append(float(resp2.q3))
            setPoint.velocities.append(0.0)
            setPoint.velocities.append(0.0)
            setPoint.velocities.append(0.0)
            setPoint.time_from_start.secs = 1
            print(setPoint)
            trajToBeSent.points.append(setPoint)


        self.gui.times = []
        self.gui.positionsq1 = []
        self.gui.positionsq2 = []
        self.gui.positionsq3 = []
        self.gui.velocitiesq1 = []
        self.gui.velocitiesq2 = []
        self.gui.velocitiesq3 = []
        self.gui.positionsx = []
        self.gui.positionsy = []
        self.gui.positionsz = []
        self.gui.velocitiesx = []
        self.gui.velocitiesy = []
        self.gui.velocitiesz = []
        self.signal.emit(1)
        rospy.sleep(1)
        self.gui.TrajControl.publish(trajToBeSent)



#verifies the 2 positions dont result in a straight line trajectory
    #in unreachable areas
    def verifyTraj(self, xCur, yCur, zCur, xDes, yDes, zDes):
        numSteps = 1000
        maxCloseness = 1.5
        ChangeX = xDes - xCur
        ChangeY = yDes - yCur
        slopeX = ChangeX/numSteps
        slopeY = ChangeY/numSteps

        for i in range(0, numSteps):
            testx = xCur+slopeX*i
            testy = yCur+slopeY*i
            if(np.sqrt(np.power(testx, 2)+np.power(testy,2))<maxCloseness):
                print("Traj Not Possible")
                return False
        print("Traj Possible")
        return True

#generates a straight line trajectory
    def createTraj(self, xCur, yCur, zCur, xDes, yDes, zDes,xVelC, yVelC, zVelC, xVel, yVel, zVel):
        points = []
        numPoints = 100 #makes 100 points to follow
        ChangeX = xDes - xCur
        ChangeY = yDes - yCur
        ChangeZ = zDes - zCur
        slopeX = ChangeX / numPoints
        slopeY = ChangeY / numPoints
        slopeZ = ChangeZ / numPoints

        TotalDist = np.sqrt(np.power(ChangeX, 2) + np.power(ChangeY, 2) + np.power(ChangeZ, 2))
        TotalSpeed = np.sqrt(np.power(xVel, 2)+ np.power(yVel, 2) + np.power(zVel, 2))
        if(TotalSpeed==0):
            timeToComplete = 5
        else:
            timeToComplete = TotalDist/TotalSpeed
        timeStep = float(timeToComplete)/float(numPoints)
        sendIK = rospy.ServiceProxy('ScaraIK', inversekinService)
        sendIKVel = rospy.ServiceProxy('ScaraIKVel', inverseVelkinService)
        resp1 = sendIK(xCur, yCur, zCur, 0, 0, 0, 1)
        setPoint = JointTrajectoryPoint()
        setPoint.positions.append(float(resp1.q1))
        setPoint.positions.append(float(resp1.q2))
        setPoint.positions.append(float(resp1.q3))
        setPoint.velocities.append(0.0)
        setPoint.velocities.append(0.0)
        setPoint.velocities.append(0.0)
        setPoint.time_from_start.secs = 0
        setPoint.time_from_start.nsecs = int(0.0001*1000000000)
        points.append(setPoint)

        XPos = xCur
        YPos = yCur
        ZPos = zCur
        for i in range(0, numPoints-2):
            XPos += slopeX
            YPos += slopeY
            ZPos += slopeZ
            resp1 = sendIK(XPos, YPos, ZPos, 0, 0, 0, 1)
            resp2 = sendIKVel(resp1.q1, resp1.q2, resp1.q3, xVel, yVel, zVel)
            setPoint = JointTrajectoryPoint()
            setPoint.positions.append(float(resp1.q1))
            setPoint.positions.append(float(resp1.q2))
            setPoint.positions.append(float(resp1.q3))
            setPoint.velocities.append(float(resp2.q1dot))
            setPoint.velocities.append(float(resp2.q2dot))
            setPoint.velocities.append(float(resp2.q3dot))
            curTime = int(timeStep * float((i + 1))*1000000000)
            curNanoSecs = int(curTime%1000000000)
            curSecs = int((curTime-curNanoSecs)/1000000000)
            setPoint.time_from_start.secs = curSecs
            setPoint.time_from_start.nsecs = curNanoSecs
            points.append(setPoint)
        setPoint = JointTrajectoryPoint()
        resp1 = sendIK(xDes, yDes, zDes, 0, 0, 0, 1)
        setPoint.positions.append(float(resp1.q1))
        setPoint.positions.append(float(resp1.q2))
        setPoint.positions.append(float(resp1.q3))
        setPoint.velocities.append(0.0)
        setPoint.velocities.append(0.0)
        setPoint.velocities.append(0.0)
        curTime = timeToComplete * 1000000000
        curNanoSecs = int(curTime % 1000000000)
        curSecs = int((curTime - curNanoSecs)/1000000000)
        setPoint.time_from_start.secs = curSecs
        setPoint.time_from_start.nsecs = curNanoSecs
        points.append(setPoint)
        return points

if __name__ == '__main__':
    rospy.init_node('Part3Controller')
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    window = P3Gui()
    window.show()
    sys.exit(app.exec_())

