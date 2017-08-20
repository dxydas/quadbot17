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
        self.paused = False
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
        runIK(target)
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
            for i in range(len(angles)):
                # Joint 2 needs its direction inverted
                if i == 1:
                    x = int( rescale(-angles[i], -180.0, 180.0, 0, 1023) )
                else:
                    x = int( rescale(angles[i], -180.0, 180.0, 0, 1023) )
                writeStr += str(i+1) + "," + str(x)
                if i < (len(angles) - 1):
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


class Joint():
    def __init__(self, id, x, y, z):
        self.id = id
        self.x = x
        self.y = y
        self.z = z


def rescale(old, oldMin, oldMax, newMin, newMax):
    oldRange = (oldMax - oldMin)
    newRange = (newMax - newMin)
    return (old - oldMin) * newRange / oldRange + newMin


def runFK(angles):
    global a
    #global angleOffsets
    global footOffset
    global T_1_in_W
    global T_2_in_W
    global T_3_in_W
    global T_4_in_W
    global T_5_in_W
    global T_F_in_W

    a = [0, 0, 29.05, 76.919, 72.96, 45.032]  # Link lengths "a-1"
    #a = [0, 0, 0, 76.919, 72.96, 0]  # Link lengths "a-1"

    #angleOffsets = [0, 0, -34, 67.5, -33.5, 0]

    footOffset = 33.596
    #footOffset = 0

    s = [0, 0, 0, 0, 0, 0]
    c = [0, 0, 0, 0, 0, 0]
    for i in range(1,6):
        #s[i] = math.sin( math.radians(angles[i-1] + angleOffsets[i]) )
        #c[i] = math.cos( math.radians(angles[i-1] + angleOffsets[i]) )
        s[i] = math.sin( math.radians(angles[i-1]) )
        c[i] = math.cos( math.radians(angles[i-1]) )

    # +90 around Y
    T_0_in_W = np.matrix( [ [  0,  0,  1,  0],
                            [  0,  1,  0,  0],
                            [ -1,  0,  0,  0],
                            [  0,  0,  0,  1] ] )

    T_1_in_0 = np.matrix( [ [ c[1], -s[1],  0, a[1]],
                            [ s[1],  c[1],  0,    0],
                            [    0,     0,  1,    0],
                            [    0,     0,  0,    1] ] )

    T_2_in_1 = np.matrix( [ [ c[2], -s[2],  0, a[2]],
                            [    0,     0, -1,    0],
                            [ s[2],  c[2],  0,    0],
                            [    0,     0,  0,    1] ] )

    T_3_in_2 = np.matrix( [ [ c[3], -s[3],  0, a[3]],
                            [ s[3],  c[3],  0,    0],
                            [    0,     0,  1,    0],
                            [    0,     0,  0,    1] ] )

    T_4_in_3 = np.matrix( [ [ c[4], -s[4],  0, a[4]],
                            [ s[4],  c[4],  0,    0],
                            [    0,     0,  1,    0],
                            [    0,     0,  0,    1] ] )

    T_5_in_4 = np.matrix( [ [ c[5], -s[5],  0, a[5]],
                            [    0,     0, -1,    0],
                            [-s[5], -c[5],  1,    0],
                            [    0,     0,  0,    1] ] )

    T_F_in_5 = np.matrix( [ [  1,  0,  0,  footOffset],
                            [  0,  1,  0,  0],
                            [  0,  0,  1,  0],
                            [  0,  0,  0,  1] ] )

    T_1_in_W = T_0_in_W * T_1_in_0
    T_2_in_W = T_1_in_W * T_2_in_1
    T_3_in_W = T_2_in_W * T_3_in_2
    T_4_in_W = T_3_in_W * T_4_in_3
    T_5_in_W = T_4_in_W * T_5_in_4
    T_F_in_W = T_5_in_W * T_F_in_5

    #print "T_F_in_W: ", T_F_in_W


def runIK(target):
    # Solve Joint 1
    num = target[1]
    den = abs(target[2]) - footOffset
    a0Rads = math.atan2(num, den)
    angles[0] = math.degrees(a0Rads) # + angleOffsets[0]

    # Lengths projected onto z-plane
    c0 = math.cos(a0Rads)
    a2p = a[2]*c0
    a3p = a[3]*c0
    a4p = a[4]*c0
    a5p = a[5]*c0

    #j4Height = abs(target[2]) - a[2] - a[5] - footOffset
    j4Height = abs(target[2]) - a2p - a5p - footOffset

    j2j4DistSquared = math.pow(j4Height, 2) + math.pow(target[0], 2)
    j2j4Dist = math.sqrt(j2j4DistSquared)
    #print "j2j4Dist: ", j2j4Dist
    #print "pow(a[3], 2): ", pow(a[3], 2)
    #print "pow(a[4], 2): ", pow(a[4], 2)
    #print "pow(j2j4Dist, 2): ", pow(j2j4Dist, 2)
    #print "num: ", (pow(a[3], 2) + pow(a[4], 2) - pow(j2j4Dist, 2))
    #print "den: ", (2*a[3]*a[4])

    # # Solve Joint 2 - Law of cosines
    # num = pow(a[3], 2) + pow(j2j4Dist, 2) - pow(a[4], 2)
    # den = 2*a[3]*j2j4Dist
    # if abs(num) <= abs(den):
    #     angles[1] = 0.0 - math.degrees( math.acos(num/den) )# - angleOffsets[1]


    num = target[0]
    den = j4Height
    psi = math.degrees( math.atan2(num, den) )


    #num = pow(a[3], 2) + j2j4DistSquared - pow(a[4], 2)
    num = pow(a3p, 2) + j2j4DistSquared - pow(a4p, 2)
    #den = 2.0*a[3]*j2j4Dist
    den = 2.0*a3p*j2j4Dist
    if abs(num) <= abs(den):
        phi = math.degrees( math.acos(num/den) )
        angles[1] = - (phi - psi)

    # Solve Joint 3 - Law of cosines
    #num = pow(a[3], 2) + pow(a[4], 2) - j2j4DistSquared
    num = pow(a3p, 2) + pow(a4p, 2) - j2j4DistSquared
    #den = 2.0*a[3]*a[4]
    den = 2.0*a3p*a4p
    if abs(num) <= abs(den):
        angles[2] = 180.0 - math.degrees( math.acos(num/den) )# - angleOffsets[2]

    # # Solve Joint 4
    # num = target[0]
    # den = abs(target[2]) - footOffset - a[5]
    # phi = math.degrees( math.atan2(num, den) )
    # num = pow(a[4], 2) + pow(j2j4Dist, 2) - pow(a[3], 2)
    # den = 2*a[4]*j2j4Dist
    # if abs(num) <= abs(den):
    #     omega = math.degrees( math.acos(num/den) )
    #     #print "phi: ", phi
    #     #print "omega: ", omega
    #     angles[3] = - (phi + omega)# + angleOffsets[3]


    #num = a[3]*abs(math.sin( math.radians(angles[1]) )) + target[0]
    #den = a[4]
    #if abs(num) <= abs(den):
    #    angles[3] = - math.degrees( math.asin(num/den) )
    #OR
    #num = pow(a[4], 2) + j2j4DistSquared - pow(a[3], 2)
    num = pow(a4p, 2) + j2j4DistSquared - pow(a3p, 2)
    #den = 2.0*a[4]*j2j4Dist
    den = 2.0*a4p*j2j4Dist
    if abs(num) <= abs(den):
        omega = math.degrees( math.acos(num/den) )
        angles[3] = - (psi + omega)# + angleOffsets[3]



    # Solve Joint 5
    angles[4] = - angles[0]# + angleOffsets[4]

    runFK(angles)

    #print "target: ", target
    #print "angles: ", angles


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
        runIK(target)
        root.after(rateMs, testIKCallback)


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

    joint1 = Joint( 1, T_1_in_W.item(0, 3), T_1_in_W.item(1, 3), T_1_in_W.item(2, 3) )
    joint2 = Joint( 2, T_2_in_W.item(0, 3), T_2_in_W.item(1, 3), T_2_in_W.item(2, 3) )
    joint3 = Joint( 3, T_3_in_W.item(0, 3), T_3_in_W.item(1, 3), T_3_in_W.item(2, 3) )
    joint4 = Joint( 4, T_4_in_W.item(0, 3), T_4_in_W.item(1, 3), T_4_in_W.item(2, 3) )
    joint5 = Joint( 5, T_5_in_W.item(0, 3), T_5_in_W.item(1, 3), T_5_in_W.item(2, 3) )
    jointFoot = Joint( 'F', T_F_in_W.item(0, 3), T_F_in_W.item(1, 3), T_F_in_W.item(2, 3) )

    drawLink(joint1, joint2)
    drawLink(joint2, joint3)
    drawLink(joint3, joint4)
    drawLink(joint4, joint5)
    drawLink(joint5, jointFoot)

    drawJoint(joint1)
    drawJoint(joint2)
    drawJoint(joint3)
    drawJoint(joint4)
    drawJoint(joint5)
    drawEE(jointFoot)

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

    topViewCanvas.create_oval( canvasW - canvasScale*joint.x - r + canvasOffset[0], canvasH + canvasScale*joint.y - r + canvasOffset[1],
                               canvasW - canvasScale*joint.x + r + canvasOffset[0], canvasH + canvasScale*joint.y + r + canvasOffset[1],
                               fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    topViewCanvas.create_text( canvasW - canvasScale*joint.x + canvasOffset[0], canvasH + canvasScale*joint.y + canvasOffset[1],
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

    topViewCanvas.create_oval( canvasW - canvasScale*joint.x - r + canvasOffset[0], canvasH + canvasScale*joint.y - r + canvasOffset[1],
                               canvasW - canvasScale*joint.x + r + canvasOffset[0], canvasH + canvasScale*joint.y + r + canvasOffset[1],
                               fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    topViewCanvas.create_text( canvasW - canvasScale*joint.x + canvasOffset[0], canvasH + canvasScale*joint.y + canvasOffset[1],
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
    topViewCanvas.create_line( canvasW - canvasScale*jointA.x + canvasOffset[0], canvasH + canvasScale*jointA.y + canvasOffset[1],
                                canvasW - canvasScale*jointB.x + canvasOffset[0], canvasH + canvasScale*jointB.y + canvasOffset[1],
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
    topViewCanvas.create_oval( canvasW - canvasScale*target[0] - r + canvasOffset[0], canvasH + canvasScale*target[1] - r + canvasOffset[1],
                               canvasW - canvasScale*target[0] + r + canvasOffset[0], canvasH + canvasScale*target[1] + r + canvasOffset[1],
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
    topViewCanvas.create_line( canvasW - canvasScale*target[0] + canvasOffset[0], canvasH + canvasScale*target[1] + canvasOffset[1],
                               canvasW - canvasScale*target[0] - speed[0]*k + canvasOffset[0],
                               canvasH + canvasScale*target[1] + speed[1]*k + canvasOffset[1],
                               fill = fillCol, width = w, tag = "clear" )


def joint1SliderCallback(val):
    angles[0] = float(val)
    runFK(angles)


def joint2SliderCallback(val):
    angles[1] = float(val)
    runFK(angles)


def joint3SliderCallback(val):
    angles[2] = float(val)
    runFK(angles)


def joint4SliderCallback(val):
    angles[3] = float(val)
    runFK(angles)


def joint5SliderCallback(val):
    angles[4] = float(val)
    runFK(angles)


def targetXSliderCallback(val):
    target[0] = targetHome[0] + float(val)/1.0
    runIK(target)


def targetYSliderCallback(val):
    target[1] = targetHome[1] + float(val)/1.0
    runIK(target)


def targetZSliderCallback(val):
    target[2] = targetHome[2] + float(val)/1.0
    runIK(target)


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
rootWidth = 2200
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

canvasW = 1060
canvasH = 760

canvasScale = 2  # 1 mm -> 2 pixels
canvasOffset = [-canvasW/2, -canvasH + 200]

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
jointSlidersFrame = Frame(controlsFrame)
targetSlidersFrame = Frame(controlsFrame)
buttonsFrame = Frame(controlsFrame)

messageBoxFrame.grid(row=0, column=0, sticky=N)
jointSlidersFrame.grid(row=0, column=1, sticky=N)
targetSlidersFrame.grid(row=0, column=2, sticky=N)
buttonsFrame.grid(row=1, column=0, sticky=N)

messageBox = Text(messageBoxFrame, width = 32, height=18, font = defaultFont)
messageBox.grid(row=0, column=0, sticky=N+S+W+E)
scrl = Scrollbar(messageBoxFrame, command=messageBox.yview)
scrl.grid(row=0, column=1, sticky=N+S)
messageBox.config(yscrollcommand=scrl.set)
messageBox.bind("<<Modified>>", messageBoxModifiedCallback)
logMessage("Started at: " + startTime)


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

tsRange = 300.0
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
joystickCheckButton.select()

testIKButton = Button(buttonsFrame, text="Test IK", command=testIK, font = defaultFont)
testIKButton.grid(row=0, column=1)

quitButton = Button(buttonsFrame, text="Quit", command=quit, font = defaultFont)
quitButton.grid(row=0, column=2)


if __name__ == '__main__':
    global angleOffsets, angles
    global targetHome, target, speed
    initViews()
    angleOffsets = [0, -34, 67.5, -33.5, 0]  # Offsets for natural "home" position
    angles = angleOffsets[:]
    target = [0, 0, 0]
    speed = [0, 0, 0]
    runFK(angles)
    joint1Slider.set(angles[0])
    joint2Slider.set(angles[1])
    joint3Slider.set(angles[2])
    joint4Slider.set(angles[3])
    joint5Slider.set(angles[4])
    targetHome = [T_F_in_W.item(0, 3), T_F_in_W.item(1, 3), T_F_in_W.item(2, 3)]
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
