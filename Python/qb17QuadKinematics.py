#!/usr/bin/env python

import sys
from std_msgs.msg import String
from Tkinter import *
from time import localtime, strftime, sleep
import math
import numpy as np
import threading
import Queue
import inputs
import serial


class App:
    def __init__(self, master):
        self.master = master
        self.poll()  # Start polling

    def poll(self):
        redraw()
        self.master.after(10, self.poll)


class GamepadReader(threading.Thread):
    def __init__(self, master):
        self.master = master
        threading.Thread.__init__(self)
        self.daemon = True
        self.gamepadOK = False
        self.gamepadUnplugged = False
        self.gamepadIOError = False

    def run(self):
        while 1:
            if not self.gamepadOK:
                self.devices = inputs.DeviceManager()
                try:
                    gamepad = self.devices.gamepads[0]
                    logMessage("Gamepad connected")
                    self.gamepadOK = True
                    self.gamepadUnplugged = False
                except IndexError:
                    self.gamepadOK = False
                    if self.gamepadUnplugged == False:
                        logMessage("Gamepad not found")
                        self.gamepadUnplugged = True
                    sleep(1)
                    continue
            try:
                # Get joystick input
                events = gamepad.read()
                for event in events:
                    self.processEvent(event)
                self.gamepadIOError = False
            except IOError:
                self.gamepadOK = False
                if self.gamepadIOError == False:
                    logMessage("Gamepad I/O error")
                    self.gamepadIOError = True
                sleep(1)
                continue

    def processEvent(self, event):
        #print(event.ev_type, event.code, event.state)
        if event.code == 'ABS_X':
            global inputLJSX
            inputLJSX = event.state
        elif event.code == 'ABS_Y':
            global inputLJSY
            inputLJSY = event.state
        elif event.code == 'ABS_RX':
            global inputRJSX
            inputRJSX = event.state
        elif event.code == 'ABS_RY':
            global inputRJSY
            inputRJSY = event.state


class GamepadHandler(threading.Thread):
    def __init__(self, master):
        self.master = master
        # Threading vars
        threading.Thread.__init__(self)
        self.daemon = True  # OK for main to exit even if instance is still running
        self.paused = True
        self.triggerPolling = True
        self.cond = threading.Condition()
        # Input vars
        self.target = targetHome[:]
        self.speed = [0, 0, 0]
        self.inputLJSXNormed = 0
        self.inputLJSYNormed = 0
        self.inputRJSXNormed = 0
        self.inputRJSYNormed = 0
        self.dt = 0.005  # 5 ms

    def run(self):
        while 1:
            with self.cond:
                if self.paused:
                    self.cond.wait()  # Block until notified
                    self.triggerPolling = True
                elif self.triggerPolling:
                    self.pollInputs()
                    self.pollIK()
                    self.triggerPolling = False

    def pause(self):
        with self.cond:
            self.paused = True

    def resume(self):
        with self.cond:
            self.paused = False
            self.cond.notify()  # Unblock self if waiting

    def pollInputs(self):
        # World X
        global inputLJSY
        self.inputLJSYNormed = self.filterInput(-inputLJSY)
        self.target[0], self.speed[0] = self.updateMotion(self.inputLJSYNormed, self.target[0], self.speed[0])
        # World Y
        global inputLJSX
        self.inputLJSXNormed = self.filterInput(-inputLJSX)
        self.target[1], self.speed[1] = self.updateMotion(self.inputLJSXNormed, self.target[1], self.speed[1])
        # World Z
        global inputRJSY
        self.inputRJSYNormed = self.filterInput(-inputRJSY)
        self.target[2], self.speed[2] = self.updateMotion(self.inputRJSYNormed, self.target[2], self.speed[2])
        with self.cond:
            if not self.paused:
                self.master.after(int(self.dt*1000), self.pollInputs)

    def pollIK(self):
        global target, speed
        target = self.target[:]
        speed = self.speed[:]
        runIK(legs[selectedLeg], target)
        with self.cond:
            if not self.paused:
                self.master.after(int(self.dt*1000), self.pollIK)

    def filterInput(self, i):
        if (i > 3277) or (i < -3277):  # ~10%
            if i > 3277:
                oldMax = 32767
            elif i < -3277:
                oldMax = 32768
            inputNormed = math.copysign(1.0, abs(i)) * rescale(i, 0, oldMax, 0, 1.0)
        else:
            inputNormed = 0
        return inputNormed

    def updateMotion(self, i, target, speed):
        mu = 1.0
        m = 1.0
        u0 = speed
        F = inputForceMax*i - dragForceCoef*u0  # Force minus linear drag
        a = F/m
        t = self.dt
        x0 = target
        # Equations of motion
        u = u0 + a*t
        x = x0 + u0*t + 0.5*a*pow(t, 2)
        # Update self
        target = x
        speed = u
        return target, speed


class SerialHandler(threading.Thread):
    def __init__(self, master):
        self.master = master
        threading.Thread.__init__(self)
        self.daemon = True
        self.cond = threading.Condition()
        # Input vars
        self.ser = 0
        self.port = "/dev/ttyUSB0"
        self.serialOK = False
        self.serialDisconnected = False
        self.dt = 0.05  # 50 ms
        self.pollSerial()

    def run(self):
        while 1:
            if not self.serialOK:
                try:
                    self.ser = serial.Serial(self.port, 38400)
                    logMessage("Serial port " + self.port + " connected")
                    self.serialOK = True
                    self.serialDisconnected = False
                except serial.SerialException:
                    self.serialOK = False
                    if self.serialDisconnected == False:
                        logMessage("Serial port " + self.port + " not connected")
                        self.serialDisconnected = True
            else:
                sleep(2)

    def pollSerial(self):
        if self.serialOK:
            writeStr = ""
            for i in range(len(legs[selectedLeg].angles)):
                # Joint 2 needs its direction inverted
                if i == 1:
                    x = int( rescale(-legs[selectedLeg].angles[i], -180.0, 180.0, 0, 1023) )
                else:
                    x = int( rescale(legs[selectedLeg].angles[i], -180.0, 180.0, 0, 1023) )
                writeStr += str(i+1) + "," + str(x)
                if i < (len(legs[selectedLeg].angles) - 1):
                    writeStr += ","
                else:
                    writeStr += "\n"
            #print "writeStr: ", writeStr
            try:
                self.ser.write(writeStr)
            except serial.SerialException:
                logMessage("Serial write error")
                self.ser.close()
                self.serialOK = False
        self.master.after(int(self.dt*1000), self.pollSerial)

    def closeSerial(self):
        if self.serialOK:
            self.ser.close()


class Leg():
    def __init__(self, id, joints, angles, tfLegBaseInRobotBase, x, y, z):
        self.id = id
        self.joints = joints
        self.angles = angles
        self.tfLegBaseInRobotBase = tfLegBaseInRobotBase
        self.x = x
        self.y = y
        self.z = z


class Joint():
    def __init__(self, id, tfJointInPrevJoint, tfJointInRobotBase, x, y, z):
        self.id = id
        self.tfJointInPrevJoint = tfJointInPrevJoint
        self.tfJointInRobotBase = tfJointInRobotBase
        self.x = x
        self.y = y
        self.z = z


def initLegs():
    # Offsets from centre of robot
    # +90 around Y
    lengthD = 150
    widthD = 50
    tfFLBaseInRobotBase = np.matrix( [ [  0,  0,  1,  lengthD],
                                       [  0,  1,  0,  widthD],
                                       [ -1,  0,  0,  0],
                                       [  0,  0,  0,  1] ] )

    tfFRBaseInRobotBase = np.matrix( [ [  0,  0,  1,  lengthD],
                                       [  0,  1,  0,  -widthD],
                                       [ -1,  0,  0,  0],
                                       [  0,  0,  0,  1] ] )

    tfRLBaseInRobotBase = np.matrix( [ [  0,  0,  1,  -lengthD],
                                       [  0,  1,  0,  widthD],
                                       [ -1,  0,  0,  0],
                                       [  0,  0,  0,  1] ] )

    tfRRBaseInRobotBase = np.matrix( [ [  0,  0,  1,  -lengthD],
                                       [  0,  1,  0,  -widthD],
                                       [ -1,  0,  0,  0],
                                       [  0,  0,  0,  1] ] )

    global legs
    legs = [0, 0, 0, 0]
    angles = [0, 0, 0, 0, 0]
    legs[0] = Leg( "FL", initJoints(), angles, tfFLBaseInRobotBase,
                   tfFLBaseInRobotBase.item(0, 3),
                   tfFLBaseInRobotBase.item(1, 3),
                   tfFLBaseInRobotBase.item(2, 3) )
    legs[1] = Leg( "FR", initJoints(), angles, tfFRBaseInRobotBase,
                   tfFRBaseInRobotBase.item(0, 3),
                   tfFRBaseInRobotBase.item(1, 3),
                   tfFRBaseInRobotBase.item(2, 3) )
    legs[2] = Leg( "RL", initJoints(), angles, tfRLBaseInRobotBase,
                   tfRLBaseInRobotBase.item(0, 3),
                   tfRLBaseInRobotBase.item(1, 3),
                   tfRLBaseInRobotBase.item(2, 3) )
    legs[3] = Leg( "RR", initJoints(), angles, tfRRBaseInRobotBase,
                   tfRRBaseInRobotBase.item(0, 3),
                   tfRRBaseInRobotBase.item(1, 3),
                   tfRRBaseInRobotBase.item(2, 3) )


def initJoints():
    tmpTF = np.matrix( [ [  1,  0,  0,  0],
                         [  0,  1,  0,  0],
                         [  0,  0,  1,  0],
                         [  0,  0,  0,  1] ] )
    joints = [0, 0, 0, 0, 0, 0]
    for j in range(0,5):
        joints[j] = Joint(j+1, tmpTF, tmpTF, 0, 0, 0)
    joints[5] = Joint("F", tmpTF, tmpTF, 0, 0, 0)  # Foot
    return joints


def rescale(old, oldMin, oldMax, newMin, newMax):
    oldRange = (oldMax - oldMin)
    newRange = (newMax - newMin)
    return (old - oldMin) * newRange / oldRange + newMin


def runFK(leg):
    global a
    global footOffset

    a = [0, 0, 29.05, 76.919, 72.96, 45.032]  # Link lengths "a-1"

    footOffset = 33.596

    s = [0, 0, 0, 0, 0, 0]
    c = [0, 0, 0, 0, 0, 0]
    for i in range(1,6):
        s[i] = math.sin( math.radians(leg.angles[i-1]) )
        c[i] = math.cos( math.radians(leg.angles[i-1]) )

    tfJointInPrevJoint = [0, 0, 0, 0, 0, 0]
    # 1 in 0
    tfJointInPrevJoint[0] = np.matrix( [ [ c[1], -s[1],  0, a[1]],
                                         [ s[1],  c[1],  0,    0],
                                         [    0,     0,  1,    0],
                                         [    0,     0,  0,    1] ] )

    tfJointInPrevJoint[1] = np.matrix( [ [ c[2], -s[2],  0, a[2]],
                                         [    0,     0, -1,    0],
                                         [ s[2],  c[2],  0,    0],
                                         [    0,     0,  0,    1] ] )

    tfJointInPrevJoint[2] = np.matrix( [ [ c[3], -s[3],  0, a[3]],
                                         [ s[3],  c[3],  0,    0],
                                         [    0,     0,  1,    0],
                                         [    0,     0,  0,    1] ] )

    tfJointInPrevJoint[3] = np.matrix( [ [ c[4], -s[4],  0, a[4]],
                                         [ s[4],  c[4],  0,    0],
                                         [    0,     0,  1,    0],
                                         [    0,     0,  0,    1] ] )

    tfJointInPrevJoint[4] = np.matrix( [ [ c[5], -s[5],  0, a[5]],
                                         [    0,     0, -1,    0],
                                         [-s[5], -c[5],  1,    0],
                                         [    0,     0,  0,    1] ] )

    tfJointInPrevJoint[5] = np.matrix( [ [  1,  0,  0,  footOffset],
                                         [  0,  1,  0,  0],
                                         [  0,  0,  1,  0],
                                         [  0,  0,  0,  1] ] )

    for j in range(0,6):
        # Assign joint transforms, in preceeding joint coords and in base coords
        leg.joints[j].tfJointInPrevJoint = tfJointInPrevJoint[j]
        if j == 0:
            leg.joints[j].tfJointInRobotBase = leg.tfLegBaseInRobotBase * tfJointInPrevJoint[0]
        else:
            leg.joints[j].tfJointInRobotBase = leg.joints[j-1].tfJointInRobotBase * tfJointInPrevJoint[j]
        # Extract joint positions in base coords
        leg.joints[j].x = leg.joints[j].tfJointInRobotBase.item(0, 3)
        leg.joints[j].y = leg.joints[j].tfJointInRobotBase.item(1, 3)
        leg.joints[j].z = leg.joints[j].tfJointInRobotBase.item(2, 3)


def runIK(leg, target):
    # Convert target in robot base to be in leg base
    # targetInLegBase = tfRobotBaseInLegBase * target
    #                 = inv(tfLegBaseInRobotBase) * target
    # Convert target to 4x1 size vector, in order to multiply with 4x4 matrix
    targetV4 = np.array([target[0], target[1], target[2], 1]).reshape(4, 1)
    targetInLegBaseV4 = np.linalg.inv(leg.tfLegBaseInRobotBase) * targetV4
    targetInLegBase = [targetInLegBaseV4.item(0), targetInLegBaseV4.item(1), targetInLegBaseV4.item(2)]

    # Solve Joint 1
    num = targetInLegBase[1]
    den = abs(targetInLegBase[0]) - footOffset
    a0Rads = math.atan2(num, den)
    leg.angles[0] = math.degrees(a0Rads)

    # Lengths projected onto z-plane
    c0 = math.cos(a0Rads)
    a2p = a[2]*c0
    a3p = a[3]*c0
    a4p = a[4]*c0
    a5p = a[5]*c0

    j4Height = abs(targetInLegBase[0]) - a2p - a5p - footOffset

    j2j4DistSquared = math.pow(j4Height, 2) + math.pow(targetInLegBase[2], 2)
    j2j4Dist = math.sqrt(j2j4DistSquared)

    # Solve Joint 2
    num = targetInLegBase[2]
    den = j4Height
    psi = math.degrees( math.atan2(num, den) )

    num = pow(a3p, 2) + j2j4DistSquared - pow(a4p, 2)
    den = 2.0*a3p*j2j4Dist
    if abs(num) <= abs(den):
        phi = math.degrees( math.acos(num/den) )
        leg.angles[1] = - (phi - psi)

    # Solve Joint 3
    num = pow(a3p, 2) + pow(a4p, 2) - j2j4DistSquared
    den = 2.0*a3p*a4p
    if abs(num) <= abs(den):
        leg.angles[2] = 180.0 - math.degrees( math.acos(num/den) )

    # # Solve Joint 4
    num = pow(a4p, 2) + j2j4DistSquared - pow(a3p, 2)
    den = 2.0*a4p*j2j4Dist
    if abs(num) <= abs(den):
        omega = math.degrees( math.acos(num/den) )
        leg.angles[3] = - (psi + omega)

    # Solve Joint 5
    leg.angles[4] = - leg.angles[0]

    runFK(leg)

    #print "targetInLegBase: ", targetInLegBase
    #print "leg.angles: ", leg.angles


def testIK():
    global t
    global rateMs
    t = 2*math.pi
    rateMs = 50
    root.after(rateMs, testIKCallback)


def testIKCallback():
    global t
    aEll = 60
    bEll = 20
    xAdjust = 0
    yAdjust = 30
    t = t - 0.1
    if t >= 0:
        u = math.tan(t/2.0)
        u2 = math.pow(u, 2)
        x = aEll*(1 - u2) / (u2 + 1)
        y = 2*bEll*u / (u2 + 1)
        target[0] = targetHome[0] + x + xAdjust
        target[2] = targetHome[2] + y + yAdjust
        runIK(legs[selectedLeg], target)
        root.after(rateMs, testIKCallback)


def loadTargets():
    tmp=1


def toggleJoystick():
    global gamepadHandler
    if jsVar.get() == 0:
        gamepadHandler.pause()
    else:
        gamepadHandler.resume()


def initViews():
    axisW = 4
    axisL = 60
    borderDist = 40

    sideViewCanvas.create_line( canvasW - (borderDist + axisL), borderDist + axisL, canvasW - borderDist, borderDist + axisL,
                                fill = "red", width = axisW, tag = "alwaysShown" )  # x-axis
    sideViewCanvas.create_text( canvasW - (borderDist + axisL), borderDist + axisL + 20, text = "X",
                                font = 6, fill = "red", tag = "alwaysShown" )
    sideViewCanvas.create_line( canvasW - borderDist, borderDist, canvasW - borderDist, borderDist + axisL,
                                fill = "blue", width = axisW, tag = "alwaysShown" )  # z-axis
    sideViewCanvas.create_text( canvasW - borderDist + 20, borderDist, text = "Z",
                                font = 6, fill = "blue", tag = "alwaysShown" )

    frontViewCanvas.create_line( canvasW - (borderDist + axisL), borderDist + axisL, canvasW - borderDist, borderDist + axisL,
                                 fill = "green", width = axisW, tag = "alwaysShown" )  # y-axis
    frontViewCanvas.create_text( canvasW - borderDist, borderDist + axisL + 20, text = "Y",
                                 font = 6, fill = "green", tag = "alwaysShown" )
    frontViewCanvas.create_line( canvasW - (borderDist + axisL), borderDist, canvasW - (borderDist + axisL), borderDist + axisL,
                                 fill = "blue", width = axisW, tag = "alwaysShown" )  # z-axis
    frontViewCanvas.create_text( canvasW - (borderDist + axisL) - 20, borderDist, text = "Z",
                                 font = 6, fill = "blue", tag = "alwaysShown" )

    topViewCanvas.create_line( canvasW - (borderDist + axisL), borderDist, canvasW - borderDist, borderDist,
                               fill = "red", width = axisW, tag = "alwaysShown" )  # x-axis
    topViewCanvas.create_text( canvasW - (borderDist + axisL), borderDist - 20, text = "X",
                               font = 6, fill = "red", tag = "alwaysShown" )
    topViewCanvas.create_line( canvasW - borderDist, borderDist, canvasW - borderDist, borderDist + axisL,
                               fill = "green", width = axisW, tag = "alwaysShown" )  # y-axis
    topViewCanvas.create_text( canvasW - borderDist + 20, borderDist + axisL, text = "Y",
                               font = 6, fill = "green", tag = "alwaysShown" )


def redraw():
    # Redraw views
    sideViewCanvas.delete("clear")
    frontViewCanvas.delete("clear")
    topViewCanvas.delete("clear")

    for leg in legs:
        for j in range(0,5):
            drawLink(leg.joints[j], leg.joints[j+1])
        for j in range(0,5):
            drawJoint(leg.joints[j])
        drawEE(leg.joints[5])

    drawTarget(target, speed)


def drawJoint(joint):
    r = 25
    w = 6
    fillCol = "#FFFFE0"
    borderCol = "#00008B"
    sideViewCanvas.create_oval( canvasW - canvasScale*joint.x - r + canvasOffset[0], canvasH - canvasScale*joint.z - r + canvasOffset[1],
                                canvasW - canvasScale*joint.x + r + canvasOffset[0], canvasH - canvasScale*joint.z + r + canvasOffset[1],
                                fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    sideViewCanvas.create_text( canvasW - canvasScale*joint.x + canvasOffset[0], canvasH - canvasScale*joint.z + canvasOffset[1],
                                text = joint.id, font = ("Times", 12, "bold"), tag = "clear" )

    frontViewCanvas.create_oval( canvasW + canvasScale*joint.y - r + canvasOffset[0], canvasH - canvasScale*joint.z - r + canvasOffset[1],
                                 canvasW + canvasScale*joint.y + r + canvasOffset[0], canvasH - canvasScale*joint.z + r + canvasOffset[1],
                                 fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    frontViewCanvas.create_text( canvasW + canvasScale*joint.y + canvasOffset[0], canvasH - canvasScale*joint.z + canvasOffset[1],
                                 text = joint.id, font = ("Times", 12, "bold"), tag = "clear" )

    topViewCanvas.create_oval( canvasW - canvasScale*joint.x - r + canvasOffset[0], canvasH + canvasScale*joint.y - r + canvasOffset[2],
                               canvasW - canvasScale*joint.x + r + canvasOffset[0], canvasH + canvasScale*joint.y + r + canvasOffset[2],
                               fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    topViewCanvas.create_text( canvasW - canvasScale*joint.x + canvasOffset[0], canvasH + canvasScale*joint.y + canvasOffset[2],
                               text = joint.id, font = ("Times", 12, "bold"), tag = "clear" )


def drawEE(joint):
    r = 25
    fillCol = "#00008B"
    borderCol = "#00008B"
    w = 6
    sideViewCanvas.create_oval( canvasW - canvasScale*joint.x - r + canvasOffset[0], canvasH - canvasScale*joint.z - r + canvasOffset[1],
                                canvasW - canvasScale*joint.x + r + canvasOffset[0], canvasH - canvasScale*joint.z + r + canvasOffset[1],
                                fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    sideViewCanvas.create_text( canvasW - canvasScale*joint.x + canvasOffset[0], canvasH - canvasScale*joint.z + canvasOffset[1],
                                text = joint.id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )

    frontViewCanvas.create_oval( canvasW + canvasScale*joint.y - r + canvasOffset[0], canvasH - canvasScale*joint.z - r + canvasOffset[1],
                                 canvasW + canvasScale*joint.y + r + canvasOffset[0], canvasH - canvasScale*joint.z + r + canvasOffset[1],
                                 fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    frontViewCanvas.create_text( canvasW + canvasScale*joint.y + canvasOffset[0], canvasH - canvasScale*joint.z + canvasOffset[1],
                                 text = joint.id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )

    topViewCanvas.create_oval( canvasW - canvasScale*joint.x - r + canvasOffset[0], canvasH + canvasScale*joint.y - r + canvasOffset[2],
                               canvasW - canvasScale*joint.x + r + canvasOffset[0], canvasH + canvasScale*joint.y + r + canvasOffset[2],
                               fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    topViewCanvas.create_text( canvasW - canvasScale*joint.x + canvasOffset[0], canvasH + canvasScale*joint.y + canvasOffset[2],
                               text = joint.id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )


def drawLink(jointA, jointB):
    fillCol = "#00008B"
    w = 10
    sideViewCanvas.create_line( canvasW - canvasScale*jointA.x + canvasOffset[0], canvasH - canvasScale*jointA.z + canvasOffset[1],
                                canvasW - canvasScale*jointB.x + canvasOffset[0], canvasH - canvasScale*jointB.z + canvasOffset[1],
                                fill = fillCol, width = w, tag = "clear" )
    frontViewCanvas.create_line( canvasW + canvasScale*jointA.y + canvasOffset[0], canvasH - canvasScale*jointA.z + canvasOffset[1],
                                 canvasW + canvasScale*jointB.y + canvasOffset[0], canvasH - canvasScale*jointB.z + canvasOffset[1],
                                 fill = fillCol, width = w, tag = "clear" )
    topViewCanvas.create_line( canvasW - canvasScale*jointA.x + canvasOffset[0], canvasH + canvasScale*jointA.y + canvasOffset[2],
                                canvasW - canvasScale*jointB.x + canvasOffset[0], canvasH + canvasScale*jointB.y + canvasOffset[2],
                                fill = fillCol, width = w, tag = "clear" )


def drawTarget(target, speed):
    r = 32
    borderCol = "#3D9140"
    w = 10
    # Target circle
    sideViewCanvas.create_oval( canvasW - canvasScale*target[0] - r + canvasOffset[0], canvasH - canvasScale*target[2] - r + canvasOffset[1],
                                canvasW - canvasScale*target[0] + r + canvasOffset[0], canvasH - canvasScale*target[2] + r + canvasOffset[1],
                                outline = borderCol, width = w, tag = "clear" )
    frontViewCanvas.create_oval( canvasW + canvasScale*target[1] - r + canvasOffset[0], canvasH - canvasScale*target[2] - r + canvasOffset[1],
                                 canvasW + canvasScale*target[1] + r + canvasOffset[0], canvasH - canvasScale*target[2] + r + canvasOffset[1],
                                 outline = borderCol, width = w, tag = "clear" )
    topViewCanvas.create_oval( canvasW - canvasScale*target[0] - r + canvasOffset[0], canvasH + canvasScale*target[1] - r + canvasOffset[2],
                               canvasW - canvasScale*target[0] + r + canvasOffset[0], canvasH + canvasScale*target[1] + r + canvasOffset[2],
                               outline = borderCol, width = w, tag = "clear" )
    # Speed vector
    fillCol = borderCol
    k = 1000.0 / inputForceMax  # Arbitrary scaling, to make max. length of vector constant
    sideViewCanvas.create_line( canvasW - canvasScale*target[0] + canvasOffset[0], canvasH - canvasScale*target[2] + canvasOffset[1],
                                canvasW - canvasScale*target[0] - speed[0]*k + canvasOffset[0],
                                canvasH - canvasScale*target[2] - speed[2]*k + canvasOffset[1],
                                fill = fillCol, width = w, tag = "clear" )
    frontViewCanvas.create_line( canvasW + canvasScale*target[1] + canvasOffset[0], canvasH - canvasScale*target[2] + canvasOffset[1],
                                 canvasW + canvasScale*target[1] + speed[1]*k + canvasOffset[0],
                                 canvasH - canvasScale*target[2] - speed[2]*k + canvasOffset[1],
                                 fill = fillCol, width = w, tag = "clear" )
    topViewCanvas.create_line( canvasW - canvasScale*target[0] + canvasOffset[0], canvasH + canvasScale*target[1] + canvasOffset[2],
                               canvasW - canvasScale*target[0] - speed[0]*k + canvasOffset[0],
                               canvasH + canvasScale*target[1] + speed[1]*k + canvasOffset[2],
                               fill = fillCol, width = w, tag = "clear" )


def selectLeg():
	global selectedLeg
	selectedLeg = rbVar.get()


def joint1SliderCallback(val):
    legs[selectedLeg].angles[0] = float(val)
    runFK(legs[selectedLeg])


def joint2SliderCallback(val):
    legs[selectedLeg].angles[1] = float(val)
    runFK(legs[selectedLeg])


def joint3SliderCallback(val):
    legs[selectedLeg].angles[2] = float(val)
    runFK(legs[selectedLeg])


def joint4SliderCallback(val):
    legs[selectedLeg].angles[3] = float(val)
    runFK(legs[selectedLeg])


def joint5SliderCallback(val):
    legs[selectedLeg].angles[4] = float(val)
    runFK(legs[selectedLeg])


def targetXSliderCallback(val):
    target[0] = targetHome[0] + float(val)/1.0
    runIK(legs[selectedLeg], target)


def targetYSliderCallback(val):
    target[1] = targetHome[1] + float(val)/1.0
    runIK(legs[selectedLeg], target)


def targetZSliderCallback(val):
    target[2] = targetHome[2] + float(val)/1.0
    runIK(legs[selectedLeg], target)


def messageBoxModifiedCallback(self):
    messageBox.see(END)
    messageBox.edit_modified(False)


def logMessage(msg):
    messageBox.insert(END, msg + "\n")


def quit():
    serialHandler.closeSerial()
    root.destroy()


global sideViewCanvas, frontViewCanvas, topViewCanvas
global canvasW, canvasH
global canvasScale, canvasOffset

startTime = strftime("%a, %d %b %Y %H:%M:%S", localtime())

root = Tk()
root.title("Quadbot 17 Kinematics")
rootWidth = 2400
rootHeight = 1660
root.geometry("%dx%d" % (rootWidth, rootHeight))


# Scaling for 4K screens
root.tk.call('tk', 'scaling', 4.0)
defaultFont = '12'


Grid.rowconfigure(root, 0, weight=1)
Grid.columnconfigure(root, 0, weight=1)

sideViewFrame = Frame(root)
topViewFrame = Frame(root)
frontViewFrame = Frame(root)
controlsFrame = Frame(root)

sideViewFrame.grid(row=0, column=0, sticky=N+W)
frontViewFrame.grid(row=0, column=1, sticky=N+E)
topViewFrame.grid(row=1, column=0, sticky=S+W)
controlsFrame.grid(row=1, column=1, sticky=S+E)

canvasW = 1170
canvasH = 760

canvasScale = 2  # 1 mm -> 2 pixels
canvasOffset = [-canvasW/2, -canvasH + 200, -canvasH + 370]  # 3rd offset is for top view only

sideViewLabel = Label(sideViewFrame, text="Side View", font = defaultFont)
sideViewLabel.grid(row=0, column=0)
sideViewCanvas = Canvas(sideViewFrame, background="#E0FFFF", width = canvasW, height = canvasH)
sideViewCanvas.grid(row=1, column=0, sticky=N+S+W+E)

frontViewLabel = Label(frontViewFrame, text="Front View", font = defaultFont)
frontViewLabel.grid(row=0, column=0)
frontViewCanvas = Canvas(frontViewFrame, background="#FFFACD", width = canvasW, height = canvasH)
frontViewCanvas.grid(row=1, column=0, sticky=N+S+W+E)

topViewLabel = Label(topViewFrame, text="Top View", font = defaultFont)
topViewLabel.grid(row=0, column=0)
topViewCanvas = Canvas(topViewFrame, background="#E0EEE0", width = canvasW, height = canvasH)
topViewCanvas.grid(row=1, column=0, sticky=N+S+W+E)

messageBoxFrame = Frame(controlsFrame)
legSelectFrame = Frame(controlsFrame)
jointSlidersFrame = Frame(controlsFrame)
targetSlidersFrame = Frame(controlsFrame)
buttonsFrame = Frame(controlsFrame)

messageBoxFrame.grid(row=0, column=0, sticky=N)
legSelectFrame.grid(row=0, column=1, sticky=N)
jointSlidersFrame.grid(row=0, column=2, sticky=N)
targetSlidersFrame.grid(row=0, column=3, sticky=N)
buttonsFrame.grid(row=1, column=0, sticky=N)

messageBox = Text(messageBoxFrame, width = 32, height=18, font = defaultFont)
messageBox.grid(row=0, column=0, sticky=N+S+W+E)
scrl = Scrollbar(messageBoxFrame, command=messageBox.yview)
scrl.grid(row=0, column=1, sticky=N+S)
messageBox.config(yscrollcommand=scrl.set)
messageBox.bind("<<Modified>>", messageBoxModifiedCallback)
logMessage("Started at: " + startTime)


legSelectLabel = Label(legSelectFrame, text="Leg", font = 6)
legSelectLabel.grid(row=0, column=0)

rbVar = IntVar()
FLRadiobutton = Radiobutton( legSelectFrame, text = "FL", font = 6, variable = rbVar, value = 0, command = selectLeg )
FRRadiobutton = Radiobutton( legSelectFrame, text = "FR", font = 6, variable = rbVar, value = 1, command = selectLeg )
RLRadiobutton = Radiobutton( legSelectFrame, text = "RL", font = 6, variable = rbVar, value = 2, command = selectLeg )
RRRadiobutton = Radiobutton( legSelectFrame, text = "RR", font = 6, variable = rbVar, value = 3, command = selectLeg )
FLRadiobutton.grid(row=1, column=0)
FRRadiobutton.grid(row=2, column=0)
RLRadiobutton.grid(row=3, column=0)
RRRadiobutton.grid(row=4, column=0)
FLRadiobutton.select()  # Set default


fkLabel = Label(jointSlidersFrame, text="FK - Joints", font = 6)
fkLabel.grid(row=0, column=0)

jsRange = 180.0
joint1Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j1",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint1SliderCallback )
joint1Slider.grid(row=1, column=0)

joint2Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j2",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint2SliderCallback )
joint2Slider.grid(row=2, column=0)

joint3Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j3",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint3SliderCallback )
joint3Slider.grid(row=3, column=0)

joint4Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j4",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint4SliderCallback )
joint4Slider.grid(row=4, column=0)

joint5Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j5",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint5SliderCallback )
joint5Slider.grid(row=5, column=0)


ikLabel = Label(targetSlidersFrame, text="IK - Target", font = 6)
ikLabel.grid(row=0, column=0)

tsRange = 500.0
targetXSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "X",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = targetXSliderCallback )
targetXSlider.grid(row=1, column=0)

targetYSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Y",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = targetYSliderCallback )
targetYSlider.grid(row=2, column=0)

targetZSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Z",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = targetZSliderCallback )
targetZSlider.grid(row=3, column=0)


jsVar = IntVar()
joystickCheckButton = Checkbutton(buttonsFrame, text="Joystick", var=jsVar, command=toggleJoystick, font = defaultFont)
joystickCheckButton.grid(row=0, column=0)
#joystickCheckButton.select()  # Set default

testIKButton = Button(buttonsFrame, text="Test IK", command=testIK, font = defaultFont)
testIKButton.grid(row=0, column=1)

loadTargetsButton = Button(buttonsFrame, text="Load", command=loadTargets, font = defaultFont)
loadTargetsButton.grid(row=0, column=2)

quitButton = Button(buttonsFrame, text="Quit", command=quit, font = defaultFont)
quitButton.grid(row=0, column=3)


if __name__ == '__main__':
    global selectedLeg
    global angleOffsets
    global targetHome, target, speed
    initLegs()
    initViews()
    selectedLeg = 0
    angleOffsets = [0, -34, 67.5, -33.5, 0]  # Offsets for natural "home" position
    target = [0, 0, 0]
    speed = [0, 0, 0]
    for leg in legs:
        leg.angles = angleOffsets[:]
        runFK(leg)
    joint1Slider.set(legs[selectedLeg].angles[0])
    joint2Slider.set(legs[selectedLeg].angles[1])
    joint3Slider.set(legs[selectedLeg].angles[2])
    joint4Slider.set(legs[selectedLeg].angles[3])
    joint5Slider.set(legs[selectedLeg].angles[4])

    # Target: Foot in robot base
    targetHome = [ legs[selectedLeg].joints[5].x, legs[selectedLeg].joints[5].y, legs[selectedLeg].joints[5].z ]
    target = targetHome[:]

    global inputLJSX
    inputLJSX = 0
    global inputLJSY
    inputLJSY = 0
    global inputRJSX
    inputRJSX = 0
    global inputRJSY
    inputRJSY = 0

    global inputForceMax, dragForceCoef
    inputForceMax = 2000
    dragForceCoef = 10

    gamepadReader = GamepadReader(root)
    gamepadReader.start()

    global gamepadHandler
    gamepadHandler = GamepadHandler(root)
    gamepadHandler.start()

    serialHandler = SerialHandler(root)
    serialHandler.start()

    App(root)
    root.mainloop()
