#!/usr/bin/env python

import sys
from std_msgs.msg import String
from Tkinter import *
from time import time, localtime, strftime, sleep
import math
import numpy as np
import threading
import Queue
import inputs
from pynput import keyboard
import serial
import csv
from copy import deepcopy


class App:
    def __init__(self, master):
        self.master = master
        self.dt = 0.01  # 10 ms
        self.prevTime = time()
        self.currTime = time()
        self.poll()  # Start polling

    def poll(self):
        self.currTime = time()
        #print "App time diff.", self.currTime - self.prevTime
        redraw()
        self.prevTime = self.currTime
        self.master.after(int(self.dt*1000), self.poll)


class GamepadReader(threading.Thread):
    def __init__(self, master):
        self.master = master
        threading.Thread.__init__(self)
        self.terminate = False
        self.gamepadOK = False
        self.gamepadUnplugged = False
        self.gamepadIOError = False

    def stop(self):
        self.terminate = True
        self._Thread__stop()

    def run(self):
        while not self.terminate:
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
        global inputModeSelect, inputLJSX, inputLJSY, inputRJSX, inputRJSY
        if event.code == 'KEY_A':
            inputModeSelect = (inputModeSelect + 1) % 3
        elif event.code == 'ABS_X':
            inputLJSX = event.state
        elif event.code == 'ABS_Y':
            inputLJSY = event.state
        elif event.code == 'ABS_RX':
            inputRJSX = event.state
        elif event.code == 'ABS_RY':
            inputRJSY = event.state


class KeyboardListener(threading.Thread):
    def __init__(self, master):
        self.master = master
        threading.Thread.__init__(self)
        self.terminate = False

    def on_press(self, key):
        global inputModeSelect, inputKBX, inputKBY, inputKBZ
        if key == keyboard.Key.space:
            inputModeSelect = (inputModeSelect + 1) % 3
        elif key == keyboard.Key.right:
            inputKBX = 32767
        elif key == keyboard.Key.left:
            inputKBX = -32768
        elif key == keyboard.Key.up:
            inputKBY = -32768
        elif key == keyboard.Key.down:
            inputKBY = 32767
        elif key == keyboard.Key.shift:
            inputKBZ = -32768
        elif key == keyboard.Key.ctrl:
            inputKBZ = 32767

    def on_release(self, key):
        global inputKBX, inputKBY, inputKBZ
        if (key == keyboard.Key.right) or (key == keyboard.Key.left):
            inputKBX = 0
        elif (key == keyboard.Key.up) or (key == keyboard.Key.down):
            inputKBY = 0
        elif (key == keyboard.Key.shift) or (key == keyboard.Key.ctrl):
            inputKBZ = 0

    def stop(self):
        self.terminate = True
        self._Thread__stop()

    def run(self):
        # Collect events until released
        with keyboard.Listener(
                on_press = self.on_press,
                on_release = self.on_release) as listener:
            listener.join()


class InputHandler(threading.Thread):
    def __init__(self, master):
        self.master = master
        # Threading vars
        threading.Thread.__init__(self)
        self.terminate = False
        self.paused = True
        self.triggerPolling = True
        self.cond = threading.Condition()
        # Input vars
        self.target = deepcopy(targetsHome[selectedLeg])
        self.speed = [0, 0, 0]
        self.inputLJSXNormed = 0
        self.inputLJSYNormed = 0
        self.inputRJSXNormed = 0
        self.inputRJSYNormed = 0
        self.dt = 0.05  # 50 ms
        # TODO: Find out why lower dt values cause program to crash, when
        #       toggling pause (does not occur in qb17Kinematics.py).
        #       It might have something to do with pollIK() ...
        self.prevTimeInputs = time()
        self.currTimeInputs = time()
        self.prevTimeIK = time()
        self.currTimeIK = time()

    def stop(self):
        self.terminate = True
        self._Thread__stop()

    def run(self):
        while not self.terminate:
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
        self.currTimeInputs = time()
        #print "Poll Inputs time diff.", self.currTimeInputs - self.prevTimeInputs
        if selectedInput == 0:
            # Keyboard
            self.inputLJSXNormed = self.filterInput(-inputKBX)
            self.inputLJSYNormed = self.filterInput(-inputKBY)
            self.inputRJSYNormed = self.filterInput(-inputKBZ)
        else:
            # Joystick
            self.inputLJSXNormed = self.filterInput(-inputLJSX)
            self.inputLJSYNormed = self.filterInput(-inputLJSY)
            self.inputRJSYNormed = self.filterInput(-inputRJSY)
        if inputModeSelect == 0:
            # World X
            self.target[0, 3], self.speed[0] = self.updateMotion(self.inputLJSYNormed, self.target[0, 3], self.speed[0])
            # World Y
            self.target[1, 3], self.speed[1] = self.updateMotion(self.inputLJSXNormed, self.target[1, 3], self.speed[1])
            # World Z
            self.target[2, 3], self.speed[2] = self.updateMotion(self.inputRJSYNormed, self.target[2, 3], self.speed[2])
        elif inputModeSelect == 1:
            pass
        elif inputModeSelect == 2:
            pass
        self.prevTimeInputs = self.currTimeInputs
        with self.cond:
            if not self.paused:
                self.master.after(int(self.dt*1000), self.pollInputs)

    def pollIK(self):
        self.currTimeIK = time()
        #print "Poll IK time diff.", self.currTimeIK - self.prevTimeIK
        global targets, speeds
        targets[selectedLeg] = deepcopy(self.target)
        speeds[selectedLeg] = deepcopy(self.speed)
        runLegIK(legs[selectedLeg], targets[selectedLeg])
        self.prevTimeIK = self.currTimeIK
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
        t = self.currTimeInputs - self.prevTimeInputs
        # Zero t if it's too large
        if t > 0.5:
            t = 0.0
        x0 = target
        # Equations of motion
        u = u0 + a*t
        x = x0 + u0*t + 0.5*a*math.pow(t, 2)
        # Update self
        target = x
        speed = u
        return target, speed


class SerialHandler(threading.Thread):
    def __init__(self, master):
        self.master = master
        threading.Thread.__init__(self)
        self.terminate = False
        self.cond = threading.Condition()
        # Input vars
        self.ser = 0
        self.port = "/dev/ttyUSB0"
        self.serialOK = False
        self.serialDisconnected = False
        self.dt = 0.05  # 50 ms
        self.prevTime = time()
        self.currTime = time()
        self.pollSerial()

    def stop(self):
        self.terminate = True
        self._Thread__stop()

    def run(self):
        while not self.terminate:
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
        self.currTime = time()
        #print "Poll Serial time diff.", self.currTime - self.prevTime
        if self.serialOK:
            writeStr = ""
            for i, leg in enumerate(legs):
                #print "leg:", leg.id, "angles:", leg.angles
                for j in range(len(leg.angles)):
                    if (i % 2 == 0):
                        # Left side
                        # Joint 2 needs its direction inverted
                        if j == 1: # Joint 2 (zero-indexed)
                            x = int( rescale(-leg.angles[j], -180.0, 180.0, 0, 1023) )
                        else:
                            x = int( rescale(leg.angles[j], -180.0, 180.0, 0, 1023) )
                    else:
                        # Right side
                        # All joints except for 1 and 5 are mirrors of left side
                        if (j == 2) or (j == 3):  # Joints 3 & 4 (zero-indexed)
                            x = int( rescale(-leg.angles[j], -180.0, 180.0, 0, 1023) )
                        else:  # Joints 1, 2 & 5 (zero-indexed)
                            x = int( rescale(leg.angles[j], -180.0, 180.0, 0, 1023) )
                    writeStr += str(leg.joints[j].id) + "," + str(x)
                    if (i == 3) and ( j == (len(leg.angles) - 1) ):
                        writeStr += "\n"
                    else:
                        writeStr += ","
            #print "writeStr: ", writeStr
            try:
                self.ser.write(writeStr)
            except serial.SerialException:
                logMessage("Serial write error")
                self.ser.close()
                self.serialOK = False
        self.prevTime = self.currTime
        self.master.after(int(self.dt*1000), self.pollSerial)

    def closeSerial(self):
        if self.serialOK:
            self.ser.close()


class Spine():
    def __init__(self, id, joints, angles, tfSpineBaseInWorld):
        self.id = id
        self.joints = joints
        self.angles = angles
        self.tfSpineBaseInWorld = tfSpineBaseInWorld


class Leg():
    def __init__(self, id, joints, angles, tfLegBaseInSpineBase):
        self.id = id
        self.joints = joints
        self.angles = angles
        self.tfLegBaseInSpineBase = tfLegBaseInSpineBase


class Joint():
    def __init__(self, id, tfJointInPrevJoint, tfJointInWorld):
        self.id = id
        self.tfJointInPrevJoint = tfJointInPrevJoint
        self.tfJointInWorld = tfJointInWorld


def initSpine():
    tmpTF = identityTF()
    global spine
    spineAngles = [0, 0, 0]
    spine = Spine( "B", initSpineJoints(21), spineAngles, tmpTF )


def initSpineJoints(startingJoint):
    tmpTF = identityTF()
    joints = [0, 0, 0]
    joints[0] = Joint(startingJoint, tmpTF, tmpTF)
    joints[1] = Joint("Dummy", tmpTF, tmpTF)
    joints[2] = Joint(startingJoint + 1, tmpTF, tmpTF)
    return joints


def initLegs():
    lengthD = 100
    widthD = 50
    heightD = 10

    # TODO: Position leg bases more accurately

    # +135 around Y
    s = math.sin( 3*math.pi/4 )
    c = math.cos( 3*math.pi/4 )
    tfFLBaseInSpineBase = np.matrix( [ [  c,  0,  s,  0],
                                       [  0,  1,  0,  0],
                                       [ -s,  0,  c,  0],
                                       [  0,  0,  0,  1] ] )
    tfFLBaseInSpineBase *= np.matrix( [ [  1,  0,  0, -heightD],
                                        [  0,  1,  0,   widthD],
                                        [  0,  0,  1,  lengthD],
                                        [  0,  0,  0,        1] ] )
    # +135 around Y
    # c, s same as above
    tfFRBaseInSpineBase = np.matrix( [ [  c,  0,  s,  0],
                                       [  0,  1,  0,  0],
                                       [ -s,  0,  c,  0],
                                       [  0,  0,  0,  1] ] )
    tfFRBaseInSpineBase *= np.matrix( [ [  1,  0,  0, -heightD],
                                        [  0,  1,  0,  -widthD],
                                        [  0,  0,  1,  lengthD],
                                        [  0,  0,  0,        1] ] )

    # +90 around X
    s = math.sin( math.pi/2 )
    c = math.cos( math.pi/2 )
    T = np.matrix( [ [  1,  0,  0,  0],
                     [  0,  c, -s,  0],
                     [  0,  s,  c,  0],
                     [  0,  0,  0,  1] ] )
    # +180 around Y
    s = math.sin( math.pi )
    c = math.cos( math.pi )
    tfRLBaseInSpineBase = T * np.matrix( [ [  c,  0,  s,  0],
                                           [  0,  1,  0,  0],
                                           [ -s,  0,  c,  0],
                                           [  0,  0,  0,  1] ] )
    tfRLBaseInSpineBase *= np.matrix( [ [  1,  0,  0,        0],
                                        [  0,  1,  0,   widthD],
                                        [  0,  0,  1, -lengthD],
                                        [  0,  0,  0,        1] ] )
    # +180 around Y
    # c, s same as above
    tfRRBaseInSpineBase = T * np.matrix( [ [  c,  0,  s,  0],
                                           [  0,  1,  0,  0],
                                           [ -s,  0,  c,  0],
                                           [  0,  0,  0,  1] ] )
    tfRRBaseInSpineBase *= np.matrix( [ [  1,  0,  0,        0],
                                        [  0,  1,  0,  -widthD],
                                        [  0,  0,  1, -lengthD],
                                        [  0,  0,  0,        1] ] )

    global legs
    legs = [0, 0, 0, 0]
    angles = [0, 0, 0, 0, 0]
    sj = 1
    legs[0] = Leg( "FL", initLegJoints(sj), angles, tfFLBaseInSpineBase )
    sj += 5
    legs[1] = Leg( "FR", initLegJoints(sj), angles, tfFRBaseInSpineBase )
    sj += 5
    legs[2] = Leg( "RL", initLegJoints(sj), angles, tfRLBaseInSpineBase )
    sj += 5
    legs[3] = Leg( "RR", initLegJoints(sj), angles, tfRRBaseInSpineBase )


def initLegJoints(startingJoint):
    tmpTF = identityTF()
    joints = [0, 0, 0, 0, 0, 0]
    for j in range(0, 5):
        joints[j] = Joint(startingJoint + j, tmpTF, tmpTF)
    joints[5] = Joint("F", tmpTF, tmpTF)  # Foot
    return joints


def identityTF():
    return np.matrix( [ [ 1.0,   0,   0,   0],
                        [   0, 1.0,   0,   0],
                        [   0,   0, 1.0,   0],
                        [   0,   0,   0, 1.0] ] )


def rescale(old, oldMin, oldMax, newMin, newMax):
    oldRange = (oldMax - oldMin)
    newRange = (newMax - newMin)
    return (old - oldMin) * newRange / oldRange + newMin


def runSpineFK(spine, x, y, z, roll, pitch, yaw):
    # Spine front: In the future this can be controlled by e.g. orientation from IMU
    spine.tfSpineBaseInWorld = identityTF()

    spine.tfSpineBaseInWorld[0, 3] = x
    spine.tfSpineBaseInWorld[1, 3] = y
    spine.tfSpineBaseInWorld[2, 3] = z

    applyYawPitchRoll(spine.tfSpineBaseInWorld, yaw, pitch, roll)

    # TODO: Get this translation accurate e.g. at location of IMU
    # Translation (to get from world to robot spine)
    spine.tfSpineBaseInWorld *= np.matrix( [ [  1,  0,  0, -50],
                                             [  0,  1,  0,   0],
                                             [  0,  0,  1,   0],
                                             [  0,  0,  0,   1] ] )

    # -45 around Y (to get from world to robot spine)
    s = math.sin( -math.pi/4 )
    c = math.cos( -math.pi/4 )
    spine.tfSpineBaseInWorld *= np.matrix( [ [  c,  0,  s,   0],
                                             [  0,  1,  0,   0],
                                             [ -s,  0,  c,   0],
                                             [  0,  0,  0,   1] ] )

    d_1b = 16.975  # Dummy link offset

    s = [0, 0, 0, 0]
    c = [0, 0, 0, 0]
    for i in range(1, 4):
        s[i] = math.sin( math.radians(spine.angles[i-1]) )
        c[i] = math.cos( math.radians(spine.angles[i-1]) )

    tfJointInPrevJoint = [0, 0, 0]

    # Front spine joint
    tfJointInPrevJoint[0] = np.matrix( [ [  c[1], -s[1],     0,     0],
                                         [  s[1],  c[1],     0,     0],
                                         [     0,     0,     1,     0],
                                         [     0,     0,     0,     1] ] )

    # Dummy joint
    tfJointInPrevJoint[1] = np.matrix( [ [     1,     0,     0,     0],
                                         [     0,     1,     0,     0],
                                         [     0,     0,     1,  d_1b],
                                         [     0,     0,     0,     1] ] )

    # Rear spine joint
    tfJointInPrevJoint[2] = np.matrix( [ [  c[3], -s[3],     0,     0],
                                         [     0,     0,     1,     0],
                                         [ -s[3], -c[3],     0,     0],
                                         [     0,     0,     0,     1] ] )

    for j in range(0, 3):
        # Assign joint transforms, in preceeding joint coords and in world coords
        spine.joints[j].tfJointInPrevJoint = deepcopy(tfJointInPrevJoint[j])
        if j == 0:
            T = spine.tfSpineBaseInWorld
        else:
            T = spine.joints[j-1].tfJointInWorld
        spine.joints[j].tfJointInWorld = T * tfJointInPrevJoint[j]

    # Update legs
    for i, leg in enumerate(legs):
        runLegIK(leg, targets[i])


def runSpineIK():
    #TODO
    # ...
    #
    #runSpineFK()
    pass


def runLegFK(leg):
    s = [0, 0, 0, 0, 0]
    c = [0, 0, 0, 0, 0]
    for i in range(0, 5):
        s[i] = math.sin( math.radians(leg.angles[i]) )
        c[i] = math.cos( math.radians(leg.angles[i]) )

    tfJointInPrevJoint = [0, 0, 0, 0, 0, 0]

    tfJointInPrevJoint[0] = np.matrix( [ [  c[0], -s[0],     0,  a[0]],
                                         [  s[0],  c[0],     0,     0],
                                         [     0,     0,     1,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[1] = np.matrix( [ [  c[1], -s[1],     0,  a[1]],
                                         [     0,     0,    -1,     0],
                                         [  s[1],  c[1],     0,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[2] = np.matrix( [ [  c[2], -s[2],     0,  a[2]],
                                         [  s[2],  c[2],     0,     0],
                                         [     0,     0,     1,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[3] = np.matrix( [ [  c[3], -s[3],     0,  a[3]],
                                         [  s[3],  c[3],     0,     0],
                                         [     0,     0,     1,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[4] = np.matrix( [ [  c[4], -s[4],     0,  a[4]],
                                         [     0,     0,     1,     0],
                                         [ -s[4], -c[4],     1,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[5] = np.matrix( [ [  1,  0,  0, a[5]],
                                         [  0,  1,  0,    0],
                                         [  0,  0,  1,    0],
                                         [  0,  0,  0,    1] ] )

    for j in range(0, 6):
        # Assign joint transforms, in preceeding joint coords and in world coords
        leg.joints[j].tfJointInPrevJoint = deepcopy(tfJointInPrevJoint[j])
        if j == 0:
            if (leg.id == "FL") or (leg.id == "FR"):
                T = spine.tfSpineBaseInWorld * leg.tfLegBaseInSpineBase
            else:
                T = spine.joints[2].tfJointInWorld * leg.tfLegBaseInSpineBase
        else:
            T = leg.joints[j-1].tfJointInWorld
        leg.joints[j].tfJointInWorld = T * tfJointInPrevJoint[j]


def runLegIK(leg, target):
    # Convert target in world to be in leg base
    tfSpineBaseInLegBase = np.linalg.inv(leg.tfLegBaseInSpineBase)
    if (leg.id == "FL") or (leg.id == "FR"):
        T = spine.tfSpineBaseInWorld
        worldInSpineBase = np.linalg.inv(spine.tfSpineBaseInWorld)
    else:
        worldInSpineBase = np.linalg.inv(spine.joints[2].tfJointInWorld)
    targetInLegBase = tfSpineBaseInLegBase * worldInSpineBase * target

    Tx = targetInLegBase[0, 3]
    Ty = targetInLegBase[1, 3]
    Tz = targetInLegBase[2, 3]

    # Extract roll/pitch/yaw (as seen in World frame) from rotation matrix
    r11 = target[0, 0]
    r21 = target[1, 0]
    r31 = target[2, 0]
    r32 = target[2, 1]
    r33 = target[2, 2]
    den = math.sqrt( math.pow(r32, 2) + math.pow(r33, 2) )
    # Roll negated
    roll = - math.atan2( r32, r33 )
    pitch = math.atan2( -r31, den )
    yaw = math.atan2( r21, r11 )

    # Trig. values
    sr = math.sin(roll)
    cr = math.cos(roll)
    sp = math.sin(pitch)
    cp = math.cos(pitch)


    # Foot link projected onto axes
    #aFx = a[5]*cr*cp
    #aFy = a[5]*sr
    #aFz = a[5]*sp

    s2r = math.pow(sr, 2)
    c2r = math.pow(cr, 2)
    s2p = math.pow(sp, 2)
    c2p = math.pow(cp, 2)

    aFSq = math.pow(a[5], 2)

    eps = 1e-10
    if abs(roll) < eps:
        aFx = a[5]*cp
        aFy = 0
        aFz = a[5]*sp
    elif abs(pitch) < eps:
        aFx = a[5]*cr
        aFy = a[5]*sr
        aFz = 0
    elif abs( abs(roll) - math.pi/2 ) < eps:
        aFx = 0
        aFy = np.sign(roll)*a[5]
        aFz = 0
    elif abs( abs(pitch) - math.pi/2 ) < eps:
        aFx = 0
        aFy = a[5]*sr
        aFz = a[5]*cr
    else:
        A = math.sqrt( aFSq * c2p / (1 - s2r*s2p) )
        C = A*cr/cp

        aFx = A*cr
        aFy = A*sr
        aFz = C*sp

        #B = math.sqrt( math.pow(aFy, 2) + math.pow(aFz, 2) )


    # Solve Joint 1
    num = Ty + aFy
    den = Tx - aFx
    a0 = math.atan2(num, den)
    leg.angles[0] = math.degrees(a0)

    # Leg links projected onto z-axis
    c0 = math.cos(a0)
    a2z = a[1]*c0
    a3z = a[2]*c0
    a4z = a[3]*c0
    a5z = a[4]*c0*sp

    # Additional vars
    a5x = a[4]*c0*cp
    X = a5x + aFx
    Z = a5z + aFz
    j4Height = Tx - a2z - X
    j2j4DistSquared = math.pow(j4Height, 2) + math.pow(Tz + Z, 2)
    j2j4Dist = math.sqrt(j2j4DistSquared)

    # Solve Joint 2
    num = Tz + Z
    den = j4Height
    psi = math.degrees( math.atan2(num, den) )

    num = math.pow(a3z, 2) + j2j4DistSquared - math.pow(a4z, 2)
    den = 2.0*a3z*j2j4Dist
    if abs(num) <= abs(den):
        phi = math.degrees( math.acos(num/den) )
        leg.angles[1] = - (phi - psi)

    # Solve Joint 3
    num = math.pow(a3z, 2) + math.pow(a4z, 2) - j2j4DistSquared
    den = 2.0*a3z*a4z
    if abs(num) <= abs(den):
        leg.angles[2] = 180.0 - math.degrees( math.acos(num/den) )

    # Solve Joint 4
    num = math.pow(a4z, 2) + j2j4DistSquared - math.pow(a3z, 2)
    den = 2.0*a4z*j2j4Dist
    if abs(num) <= abs(den):
        omega = math.degrees( math.acos(num/den) )
        leg.angles[3] = - ( psi + omega + math.degrees(pitch) )

    # Solve Joint 5
    leg.angles[4] = - math.degrees(a0 + roll)

    runLegFK(leg)

    #print "target: ", target
    #print "targetInLegBase: ", targetInLegBase
    #print "leg.angles: ", leg.angles


def testIK():
    global tTIK
    global rateMsTIK
    tTIK = 2*math.pi
    rateMsTIK = 50
    root.after(rateMsTIK, testIKCallback)


def testIKCallback():
    global tTIK
    aEll = 60
    bEll = 20
    xAdjust = 0
    yAdjust = 30
    tTIK = tTIK - 0.1
    if tTIK >= 0:
        u = math.tan(tTIK/2.0)
        u2 = math.pow(u, 2)
        x = aEll*(1 - u2) / (u2 + 1)
        y = 2*bEll*u / (u2 + 1)
        targets[selectedLeg][0, 3] = targetsHome[selectedLeg][0, 3] + x + xAdjust
        targets[selectedLeg][2, 3] = targetsHome[selectedLeg][2, 3] + y + yAdjust
        runLegIK(legs[selectedLeg], targets[selectedLeg])
        root.after(rateMsTIK, testIKCallback)


def loadFromFile(filename):
    global FLUpDown
    global FLFwdBack
    global FRUpDown
    global FRFwdBack
    global RLUpDown
    global RLFwdBack
    global RRUpDown
    global RRFwdBack

    FLUpDown = []
    FLFwdBack = []
    FRUpDown = []
    FRFwdBack = []
    RLUpDown = []
    RLFwdBack = []
    RRUpDown = []
    RRFwdBack = []

    arraySize = 100
    rowOffset = 2
    upDownAmplAdjust = 50
    fwdBackamplAdjust = 40
    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for r, row in enumerate(reader):
            if r in range(rowOffset, rowOffset + arraySize):
                #print r, row
                for c, col in enumerate(row):
                    if c in range(2, 10):
                        #print c, col
                        if c == 2:
                            FLUpDown.append(upDownAmplAdjust*float(col))
                        if c == 3:
                            FLFwdBack.append(fwdBackamplAdjust*float(col))
                        if c == 4:
                            FRUpDown.append(upDownAmplAdjust*float(col))
                        if c == 5:
                            FRFwdBack.append(fwdBackamplAdjust*float(col))
                        if c == 6:
                            RLUpDown.append(upDownAmplAdjust*float(col))
                        if c == 7:
                            RLFwdBack.append(fwdBackamplAdjust*float(col))
                        if c == 8:
                            RRUpDown.append(upDownAmplAdjust*float(col))
                        if c == 9:
                            RRFwdBack.append(fwdBackamplAdjust*float(col))
    csvfile.close()


def findClosestLegPose():
    minDist = 0
    idx = 0
    for i in range(0, len(FLUpDown)):

        distances = np.zeros(len(FLUpDown))
        minThresh = 20
        penalty = 10

        x = abs(currentPose[0] - FLUpDown[i])
        distances[0] = x
        if (x > minThresh):
            distances[0] += penalty

        x = abs(currentPose[1] - FLFwdBack[i])
        distances[1] = x
        if (x > minThresh):
            distances[1] += penalty

        x = abs(currentPose[2] - FRUpDown[i])
        distances[2] = x
        if (x > minThresh):
            distances[2] += penalty

        x = abs(currentPose[3] - FRFwdBack[i])
        distances[3] = x
        if (x > minThresh):
            distances[3] += penalty

        x = abs(currentPose[4] - RLUpDown[i])
        distances[4] = x
        if (x > minThresh):
            distances[4] += penalty

        x = abs(currentPose[5] - RLFwdBack[i])
        distances[5] = x
        if (x > minThresh):
            distances[5] += penalty

        x = abs(currentPose[6] - RRUpDown[i])
        distances[6] = x
        if (x > minThresh):
            distances[6] += penalty

        x = abs(currentPose[7] - RRFwdBack[i])
        distances[7] = x
        if (x > minThresh):
            distances[7] += penalty

        distanceMetric = 0
        for d in distances:
            distanceMetric += d

        if (i == 0) or (distanceMetric < minDist):
            minDist = distanceMetric
            idx = i

    #print "Current index:", iLT
    #print "Closest new index:", idx
    #print "Dist:", minDist
    #print "Index diff (abs):", abs(iLT - idx)

    return idx


def loadTargets1():
    # Load from csv
    loadFromFile("Gait_Creep.csv")

    # Run IK
    global iLT
    global rateMsLT
    global gaitCallbackRunning
    rateMsLT = 30
    if not 'gaitCallbackRunning' in globals():
        gaitCallbackRunning = False
    if not gaitCallbackRunning:
        iLT = 0
        root.after(rateMsLT, loadTargetsCallback)
    else:
        iLT = findClosestLegPose()


def loadTargets2():
    # Load from csv
    loadFromFile("Gait_Walk.csv")

    # Run IK
    global iLT
    global rateMsLT
    global gaitCallbackRunning
    rateMsLT = 30
    if not 'gaitCallbackRunning' in globals():
        gaitCallbackRunning = False
    if not gaitCallbackRunning:
        iLT = 0
        root.after(rateMsLT, loadTargetsCallback)
    else:
        iLT = findClosestLegPose()


def loadTargetsCallback():
    global showTargets
    global iLT
    global currentPose
    global gaitCallbackRunning
    showTargets = False
    xAdjust = -20
    zAdjust = 20
    #print "i: ", iLT

    if iLT < len(FLUpDown):
        # FL
        i = 0
        targets[i][0, 3] = targetsHome[i][0, 3] + FLFwdBack[iLT] + xAdjust
        targets[i][1, 3] = targetsHome[i][1, 3]
        targets[i][2, 3] = targetsHome[i][2, 3] + FLUpDown[iLT] + zAdjust
        runLegIK(legs[i], targets[i])
        # FR
        i = 1
        targets[i][0, 3] = targetsHome[i][0, 3] + FRFwdBack[iLT] + xAdjust
        targets[i][1, 3] = targetsHome[i][1, 3]
        targets[i][2, 3] = targetsHome[i][2, 3] + FRUpDown[iLT] + zAdjust
        runLegIK(legs[i], targets[i])
        # RL
        i = 2
        targets[i][0, 3] = targetsHome[i][0, 3] + RLFwdBack[iLT] + xAdjust
        targets[i][1, 3] = targetsHome[i][1, 3]
        targets[i][2, 3] = targetsHome[i][2, 3] + RLUpDown[iLT] + zAdjust
        runLegIK(legs[i], targets[i])
        # RR
        i = 3
        targets[i][0, 3] = targetsHome[i][0, 3] + RRFwdBack[iLT] + xAdjust
        targets[i][1, 3] = targetsHome[i][1, 3]
        targets[i][2, 3] = targetsHome[i][2, 3] + RRUpDown[iLT] + zAdjust
        runLegIK(legs[i], targets[i])

        currentPose = [ FLUpDown[iLT], FLFwdBack[iLT], FRUpDown[iLT], FRFwdBack[iLT],
                        RLUpDown[iLT], RLFwdBack[iLT], RRUpDown[iLT], RRFwdBack[iLT] ]

        iLT = iLT + 1
        gaitCallbackRunning = True
        root.after(rateMsLT, loadTargetsCallback)

    else:
        #print "Done"
        gaitCallbackRunning = False
        showTargets = True


def initViews():
    axisW = scsz*2
    axisL = scsz*30
    borderDist = scsz*20

    # Side view axis widget
    sideViewCanvas.create_line( canvasW - (borderDist + axisL), borderDist + axisL, canvasW - borderDist, borderDist + axisL,
                                fill = "red", width = axisW, tag = "alwaysShown" )  # x-axis
    sideViewCanvas.create_text( canvasW - (borderDist + axisL), borderDist + axisL + scsz*10, text = "X",
                                font = defaultFont, fill = "red", tag = "alwaysShown" )
    sideViewCanvas.create_line( canvasW - borderDist, borderDist, canvasW - borderDist, borderDist + axisL,
                                fill = "blue", width = axisW, tag = "alwaysShown" )  # z-axis
    sideViewCanvas.create_text( canvasW - borderDist + scsz*10, borderDist, text = "Z",
                                font = defaultFont, fill = "blue", tag = "alwaysShown" )

    # Front view axis widget
    frontViewCanvas.create_line( canvasW - (borderDist + axisL), borderDist + axisL, canvasW - borderDist, borderDist + axisL,
                                 fill = "green", width = axisW, tag = "alwaysShown" )  # y-axis
    frontViewCanvas.create_text( canvasW - borderDist, borderDist + axisL + scsz*10, text = "Y",
                                 font = defaultFont, fill = "green", tag = "alwaysShown" )
    frontViewCanvas.create_line( canvasW - (borderDist + axisL), borderDist, canvasW - (borderDist + axisL), borderDist + axisL,
                                 fill = "blue", width = axisW, tag = "alwaysShown" )  # z-axis
    frontViewCanvas.create_text( canvasW - (borderDist + axisL) - scsz*10, borderDist, text = "Z",
                                 font = defaultFont, fill = "blue", tag = "alwaysShown" )

    # Top view axis widget
    topViewCanvas.create_line( canvasW - (borderDist + axisL), borderDist, canvasW - borderDist, borderDist,
                               fill = "red", width = axisW, tag = "alwaysShown" )  # x-axis
    topViewCanvas.create_text( canvasW - (borderDist + axisL), borderDist - scsz*10, text = "X",
                               font = defaultFont, fill = "red", tag = "alwaysShown" )
    topViewCanvas.create_line( canvasW - borderDist, borderDist, canvasW - borderDist, borderDist + axisL,
                               fill = "green", width = axisW, tag = "alwaysShown" )  # y-axis
    topViewCanvas.create_text( canvasW - borderDist + scsz*10, borderDist + axisL, text = "Y",
                               font = defaultFont, fill = "green", tag = "alwaysShown" )

    # Origin point on canvas
    r = scsz*3
    fillCol = "black"
    borderCol = "black"
    w = scsz*1
    sideViewCanvas.create_oval( canvasW - r + canvasOffset[0], canvasH - r + canvasOffset[1],
                                canvasW + r + canvasOffset[0], canvasH + r + canvasOffset[1],
                                fill = fillCol, outline = borderCol, width = w, tag = "alwaysShown" )


def redraw():
    # Redraw views
    sideViewCanvas.delete("clear")
    frontViewCanvas.delete("clear")
    topViewCanvas.delete("clear")

    # Spine
    for j in range(2, -1, -2):  # Skip dummy joint
        drawJoint( spine.joints[j].id,
                   spine.joints[j].tfJointInWorld[0, 3],
                   spine.joints[j].tfJointInWorld[1, 3],
                   spine.joints[j].tfJointInWorld[2, 3] )

    # Legs
    for leg in reversed(legs):
        for j in range(4, -1, -1):
            drawLink( leg.joints[j].tfJointInWorld[0, 3],
                      leg.joints[j].tfJointInWorld[1, 3],
                      leg.joints[j].tfJointInWorld[2, 3],
                      leg.joints[j+1].tfJointInWorld[0, 3],
                      leg.joints[j+1].tfJointInWorld[1, 3],
                      leg.joints[j+1].tfJointInWorld[2, 3] )
        for j in range(4, -1, -1):
            drawJoint( leg.joints[j].id,
                       leg.joints[j].tfJointInWorld[0, 3],
                       leg.joints[j].tfJointInWorld[1, 3],
                       leg.joints[j].tfJointInWorld[2, 3] )
        drawEE( leg.joints[5].id,
                leg.joints[5].tfJointInWorld[0, 3],
                leg.joints[5].tfJointInWorld[1, 3],
                leg.joints[5].tfJointInWorld[2, 3] )

    # Target
    global showTargets
    if showTargets:
        for i, target in enumerate(targets):
            drawTarget( target,
                        speeds[i] )


def drawJoint(id, x, y, z):
    r = scsz*13
    fillCol = "#FFFFE0"
    borderCol = "#00008B"
    w = scsz*3
    sideViewCanvas.create_oval( canvasW - canvasScale*x - r + canvasOffset[0], canvasH - canvasScale*z - r + canvasOffset[1],
                                canvasW - canvasScale*x + r + canvasOffset[0], canvasH - canvasScale*z + r + canvasOffset[1],
                                fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    sideViewCanvas.create_text( canvasW - canvasScale*x + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                text = id, font = ("Times", 12, "bold"), tag = "clear" )

    frontViewCanvas.create_oval( canvasW + canvasScale*y - r + canvasOffset[0], canvasH - canvasScale*z - r + canvasOffset[1],
                                 canvasW + canvasScale*y + r + canvasOffset[0], canvasH - canvasScale*z + r + canvasOffset[1],
                                 fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    frontViewCanvas.create_text( canvasW + canvasScale*y + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                 text = id, font = ("Times", 12, "bold"), tag = "clear" )

    topViewCanvas.create_oval( canvasW - canvasScale*x - r + canvasOffset[0], canvasH + canvasScale*y - r + canvasOffset[2],
                               canvasW - canvasScale*x + r + canvasOffset[0], canvasH + canvasScale*y + r + canvasOffset[2],
                               fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    topViewCanvas.create_text( canvasW - canvasScale*x + canvasOffset[0], canvasH + canvasScale*y + canvasOffset[2],
                               text = id, font = ("Times", 12, "bold"), tag = "clear" )


def drawEE(id, x, y, z):
    r = scsz*13
    fillCol = "#00008B"
    borderCol = "#00008B"
    w = scsz*3
    sideViewCanvas.create_oval( canvasW - canvasScale*x - r + canvasOffset[0], canvasH - canvasScale*z - r + canvasOffset[1],
                                canvasW - canvasScale*x + r + canvasOffset[0], canvasH - canvasScale*z + r + canvasOffset[1],
                                fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    sideViewCanvas.create_text( canvasW - canvasScale*x + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                text = id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )

    frontViewCanvas.create_oval( canvasW + canvasScale*y - r + canvasOffset[0], canvasH - canvasScale*z - r + canvasOffset[1],
                                 canvasW + canvasScale*y + r + canvasOffset[0], canvasH - canvasScale*z + r + canvasOffset[1],
                                 fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    frontViewCanvas.create_text( canvasW + canvasScale*y + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                 text = id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )

    topViewCanvas.create_oval( canvasW - canvasScale*x - r + canvasOffset[0], canvasH + canvasScale*y - r + canvasOffset[2],
                               canvasW - canvasScale*x + r + canvasOffset[0], canvasH + canvasScale*y + r + canvasOffset[2],
                               fill = fillCol, outline = borderCol, width = w, tag = "clear" )
    topViewCanvas.create_text( canvasW - canvasScale*x + canvasOffset[0], canvasH + canvasScale*y + canvasOffset[2],
                               text = id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )


def drawLink(Ax, Ay, Az, Bx, By, Bz):
    fillCol = "#00008B"
    w = scsz*5
    sideViewCanvas.create_line( canvasW - canvasScale*Ax + canvasOffset[0], canvasH - canvasScale*Az + canvasOffset[1],
                                canvasW - canvasScale*Bx + canvasOffset[0], canvasH - canvasScale*Bz + canvasOffset[1],
                                fill = fillCol, width = w, tag = "clear" )
    frontViewCanvas.create_line( canvasW + canvasScale*Ay + canvasOffset[0], canvasH - canvasScale*Az + canvasOffset[1],
                                 canvasW + canvasScale*By + canvasOffset[0], canvasH - canvasScale*Bz + canvasOffset[1],
                                 fill = fillCol, width = w, tag = "clear" )
    topViewCanvas.create_line( canvasW - canvasScale*Ax + canvasOffset[0], canvasH + canvasScale*Ay + canvasOffset[2],
                                canvasW - canvasScale*Bx + canvasOffset[0], canvasH + canvasScale*By + canvasOffset[2],
                                fill = fillCol, width = w, tag = "clear" )


def drawTarget(target, speed):
    # Target circle
    r = scsz*16
    borderCol = "green"
    w = scsz*5
    x = target[0, 3]
    y = target[1, 3]
    z = target[2, 3]
    sideViewCanvas.create_oval( canvasW - canvasScale*x - r + canvasOffset[0], canvasH - canvasScale*z - r + canvasOffset[1],
                                canvasW - canvasScale*x + r + canvasOffset[0], canvasH - canvasScale*z + r + canvasOffset[1],
                                outline = borderCol, width = w, tag = "clear" )
    frontViewCanvas.create_oval( canvasW + canvasScale*y - r + canvasOffset[0], canvasH - canvasScale*z - r + canvasOffset[1],
                                 canvasW + canvasScale*y + r + canvasOffset[0], canvasH - canvasScale*z + r + canvasOffset[1],
                                 outline = borderCol, width = w, tag = "clear" )
    topViewCanvas.create_oval( canvasW - canvasScale*x - r + canvasOffset[0], canvasH + canvasScale*y - r + canvasOffset[2],
                               canvasW - canvasScale*x + r + canvasOffset[0], canvasH + canvasScale*y + r + canvasOffset[2],
                               outline = borderCol, width = w, tag = "clear" )
    # Line along X
    tmpVec = np.array([50, 0, 0, 1]).reshape(4, 1)
    tmpVec = target * tmpVec
    fillCol = "red"
    lx = tmpVec[0, 0]
    ly = tmpVec[1, 0]
    lz = tmpVec[2, 0]
    sideViewCanvas.create_line( canvasW - canvasScale*x + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                canvasW - canvasScale*lx + canvasOffset[0], canvasH - canvasScale*lz + canvasOffset[1],
                                    fill = fillCol, width = w, tag = "clear" )
    frontViewCanvas.create_line( canvasW + canvasScale*y + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                 canvasW + canvasScale*ly + canvasOffset[0], canvasH - canvasScale*lz + canvasOffset[1],
                                 fill = fillCol, width = w, tag = "clear" )
    topViewCanvas.create_line( canvasW - canvasScale*x + canvasOffset[0], canvasH + canvasScale*y + canvasOffset[2],
                               canvasW - canvasScale*lx + canvasOffset[0], canvasH + canvasScale*ly + canvasOffset[2],
                               fill = fillCol, width = w, tag = "clear" )
    # Line along Y
    tmpVec = np.array([0, 50, 0, 1]).reshape(4, 1)
    tmpVec = target * tmpVec
    fillCol = "green"
    lx = tmpVec[0, 0]
    ly = tmpVec[1, 0]
    lz = tmpVec[2, 0]
    sideViewCanvas.create_line( canvasW - canvasScale*x + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                canvasW - canvasScale*lx + canvasOffset[0], canvasH - canvasScale*lz + canvasOffset[1],
                                    fill = fillCol, width = w, tag = "clear" )
    frontViewCanvas.create_line( canvasW + canvasScale*y + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                 canvasW + canvasScale*ly + canvasOffset[0], canvasH - canvasScale*lz + canvasOffset[1],
                                 fill = fillCol, width = w, tag = "clear" )
    topViewCanvas.create_line( canvasW - canvasScale*x + canvasOffset[0], canvasH + canvasScale*y + canvasOffset[2],
                               canvasW - canvasScale*lx + canvasOffset[0], canvasH + canvasScale*ly + canvasOffset[2],
                               fill = fillCol, width = w, tag = "clear" )
    # Line along Z
    tmpVec = np.array([0, 0, 50, 1]).reshape(4, 1)
    tmpVec = target * tmpVec
    fillCol = "blue"
    lx = tmpVec[0, 0]
    ly = tmpVec[1, 0]
    lz = tmpVec[2, 0]
    sideViewCanvas.create_line( canvasW - canvasScale*x + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                canvasW - canvasScale*lx + canvasOffset[0], canvasH - canvasScale*lz + canvasOffset[1],
                                    fill = fillCol, width = w, tag = "clear" )
    frontViewCanvas.create_line( canvasW + canvasScale*y + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                 canvasW + canvasScale*ly + canvasOffset[0], canvasH - canvasScale*lz + canvasOffset[1],
                                 fill = fillCol, width = w, tag = "clear" )
    topViewCanvas.create_line( canvasW - canvasScale*x + canvasOffset[0], canvasH + canvasScale*y + canvasOffset[2],
                               canvasW - canvasScale*lx + canvasOffset[0], canvasH + canvasScale*ly + canvasOffset[2],
                               fill = fillCol, width = w, tag = "clear" )
    # Speed vector
    fillCol = "#39FF14"
    sx = speed[0]
    sy = speed[1]
    sz = speed[2]
    k = 500.0 / inputForceMax  # Arbitrary scaling, to make max. length of vector constant
    sideViewCanvas.create_line( canvasW - canvasScale*x + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                canvasW - canvasScale*x - sx*k + canvasOffset[0], canvasH - canvasScale*z - sz*k + canvasOffset[1],
                                fill = fillCol, width = w, tag = "clear" )
    frontViewCanvas.create_line( canvasW + canvasScale*y + canvasOffset[0], canvasH - canvasScale*z + canvasOffset[1],
                                 canvasW + canvasScale*y + sy*k + canvasOffset[0], canvasH - canvasScale*z - sz*k + canvasOffset[1],
                                 fill = fillCol, width = w, tag = "clear" )
    topViewCanvas.create_line( canvasW - canvasScale*x + canvasOffset[0], canvasH + canvasScale*y + canvasOffset[2],
                               canvasW - canvasScale*x - sx*k + canvasOffset[0], canvasH + canvasScale*y + sy*k + canvasOffset[2],
                               fill = fillCol, width = w, tag = "clear" )


def applyYawPitchRoll(T, yaw, pitch, roll):
    s = math.sin( math.radians(yaw) )
    c = math.cos( math.radians(yaw) )
    Rot = np.matrix( [ [  c, -s,  0,  0],
                       [  s,  c,  0,  0],
                       [  0,  0,  1,  0],
                       [  0,  0,  0,  1] ] )
    s = math.sin( math.radians(pitch) )
    c = math.cos( math.radians(pitch) )
    Rot *= np.matrix( [ [  c,  0,  s,  0],
                        [  0,  1,  0,  0],
                        [ -s,  0,  c,  0],
                        [  0,  0,  0,  1] ] )
    s = math.sin( math.radians(roll) )
    c = math.cos( math.radians(roll) )
    Rot *= np.matrix( [ [  1,  0,  0,  0],
                        [  0,  c, -s,  0],
                        [  0,  s,  c,  0],
                        [  0,  0,  0,  1] ] )
    for r in range(0, 3):
        for c in range(0, 3):
            T[r, c] = Rot[r, c]


def selectLegCallback():
    global selectedLeg
    selectedLeg = rbLegVar.get()


def joint1SliderCallback(val):
    legs[selectedLeg].angles[0] = float(val)
    runLegFK(legs[selectedLeg])


def joint2SliderCallback(val):
    legs[selectedLeg].angles[1] = float(val)
    runLegFK(legs[selectedLeg])


def joint3SliderCallback(val):
    legs[selectedLeg].angles[2] = float(val)
    runLegFK(legs[selectedLeg])


def joint4SliderCallback(val):
    legs[selectedLeg].angles[3] = float(val)
    runLegFK(legs[selectedLeg])


def joint5SliderCallback(val):
    legs[selectedLeg].angles[4] = float(val)
    runLegFK(legs[selectedLeg])


def targetXSliderCallback(val):
    targets[selectedLeg][0, 3] = targetsHome[selectedLeg][0, 3] + float(val)
    targets[selectedLeg][1, 3] = targetsHome[selectedLeg][1, 3] + float(targetYSlider.get())
    targets[selectedLeg][2, 3] = targetsHome[selectedLeg][2, 3] + float(targetZSlider.get())
    runLegIK(legs[selectedLeg], targets[selectedLeg])


def targetYSliderCallback(val):
    targets[selectedLeg][0, 3] = targetsHome[selectedLeg][0, 3] + float(targetXSlider.get())
    targets[selectedLeg][1, 3] = targetsHome[selectedLeg][1, 3] + float(val)
    targets[selectedLeg][2, 3] = targetsHome[selectedLeg][2, 3] + float(targetZSlider.get())
    runLegIK(legs[selectedLeg], targets[selectedLeg])


def targetZSliderCallback(val):
    targets[selectedLeg][0, 3] = targetsHome[selectedLeg][0, 3] + float(targetXSlider.get())
    targets[selectedLeg][1, 3] = targetsHome[selectedLeg][1, 3] + float(targetYSlider.get())
    targets[selectedLeg][2, 3] = targetsHome[selectedLeg][2, 3] + float(val)
    runLegIK(legs[selectedLeg], targets[selectedLeg])


def targetRollSliderCallback(val):
    # Roll is rotation around X
    applyYawPitchRoll( targets[selectedLeg],
                       0,#targetYawSlider.get(),
                       targetPitchSlider.get(),
                       float(val))
    runLegIK(legs[selectedLeg], targets[selectedLeg])


def targetPitchSliderCallback(val):
    # Pitch is rotation around Y
    applyYawPitchRoll( targets[selectedLeg],
                       0,#targetYawSlider.get(),
                       float(val),
                       targetRollSlider.get() )
    runLegIK(legs[selectedLeg], targets[selectedLeg])


#def targetYawSliderCallback(val):
#    # Yaw is rotation around Z
#    applyYawPitchRoll( targets[selectedLeg],
#                       float(val),
#                       targetPitchSlider.get(),
#                       targetRollSlider.get() )
#    runLegIK(legs[selectedLeg], targets[selectedLeg])


def spineXSliderCallback(val):
    x = float(val)
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    runSpineFK(spine, x, y, z, roll, pitch, yaw)


def spineYSliderCallback(val):
    x = spineXSlider.get()
    y = float(val)
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    runSpineFK(spine, x, y, z, roll, pitch, yaw)


def spineZSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = float(val)
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    runSpineFK(spine, x, y, z, roll, pitch, yaw)


def spineRollSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = float(val)
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    runSpineFK(spine, x, y, z, roll, pitch, yaw)


def spinePitchSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = float(val)
    yaw = spineYawSlider.get()
    runSpineFK(spine, x, y, z, roll, pitch, yaw)


def spineYawSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = float(val)
    runSpineFK(spine, x, y, z, roll, pitch, yaw)


def spineJoint1SliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    spine.angles[0] = float(val)
    runSpineFK(spine, x, y, z, roll, pitch, yaw)
    # Dummy adjustment while IMU is not present:
    spineRollSlider.set( (spineAngleOffsets[0] + spine.angles[0]) / 2.0 )
    spinePitchSlider.set( (spineAngleOffsets[2] - spine.angles[2]) / 2.0 )


def spineJoint2SliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    spine.angles[2] = float(val)
    runSpineFK(spine, x, y, z, roll, pitch, yaw)
    # Dummy adjustment while IMU is not present
    spineRollSlider.set( (spineAngleOffsets[0] + spine.angles[0]) / 2.0 )
    spinePitchSlider.set( (spineAngleOffsets[2] - spine.angles[2]) / 2.0 )


def toggleInput():
    if toggleIpVar.get() == 0:
        inputHandler.pause()
    else:
        inputHandler.resume()


def selectInput():
    global selectedInput
    selectedInput = rbIpVar.get()


def messageBoxModifiedCallback(self):
    messageBox.see(END)
    messageBox.edit_modified(False)


def logMessage(msg):
    messageBox.insert(END, msg + "\n")


def quit():
    serialHandler.closeSerial()
    gamepadReader.stop()
    inputHandler.stop()
    serialHandler.stop()
    keyboardListener.stop()
    # Wait for threads to finish
    #print threading.active_count()
    while gamepadReader.isAlive() or inputHandler.isAlive() or serialHandler.isAlive():
        #print "waiting"
        sleep(0.1)
    #print threading.active_count()
    root.destroy()


global scsz
global sideViewCanvas, frontViewCanvas, topViewCanvas
global canvasW, canvasH
global canvasScale, canvasOffset
global targetXSlider, targetYSlider, targetZSlider

startTime = strftime("%a, %d %b %Y %H:%M:%S", localtime())

# Screen size var
# For HD screen, use 1
# For 4K screen, use 2
scsz = 1

root = Tk()
root.title("Quadbot 17 Kinematics")
rootWidth = scsz*1420
rootHeight = scsz*830
root.geometry("%dx%d" % (rootWidth, rootHeight))


defaultFont = ("System", 12)


# Scaling for 4K screens
if scsz == 2:
    root.tk.call('tk', 'scaling', 4.0)


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

canvasW = scsz*585
canvasH = scsz*380

canvasScale = scsz  # 1 mm -> scsz pixels
canvasOffset = [-canvasW/2, -canvasH + scsz*100, -canvasH + scsz*185]  # 3rd offset is for top view only

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

controlsSubFrame = Frame(controlsFrame)
buttonsFrame = Frame(controlsFrame)

messageBoxFrame = Frame(controlsSubFrame)
selectFrame = Frame(controlsSubFrame)
jointSlidersFrame = Frame(controlsSubFrame)
targetSlidersFrame = Frame(controlsSubFrame)
spineSlidersFrame = Frame(controlsSubFrame)

messageBoxFrame.grid(row=0, column=0, sticky=N)
selectFrame.grid(row=0, column=1, sticky=N)
jointSlidersFrame.grid(row=0, column=2, sticky=N)
targetSlidersFrame.grid(row=0, column=3, sticky=N)
spineSlidersFrame.grid(row=0, column=4, sticky=N)

legSelectSubFrame = Frame(selectFrame)
legSelectSubFrame.grid(row=0, column=0, sticky=N)

controlsSubFrame.grid(row=0, column=0, sticky=N)
buttonsFrame.grid(row=1, column=0, sticky=N)

messageBox = Text(messageBoxFrame, width = 32, height=18, font = defaultFont)
messageBox.grid(row=0, column=0, sticky=N+S+W+E)
scrl = Scrollbar(messageBoxFrame, command=messageBox.yview)
scrl.grid(row=0, column=1, sticky=N+S)
messageBox.config(yscrollcommand=scrl.set)
messageBox.bind("<<Modified>>", messageBoxModifiedCallback)
logMessage("Started at: " + startTime)


legSelectLabel = Label(legSelectSubFrame, text="Leg", font = defaultFont)
legSelectLabel.grid(row=0, column=0)

rbLegVar = IntVar()
FLRadiobutton = Radiobutton( legSelectSubFrame, text = "FL", font = defaultFont, variable = rbLegVar,
                             value = 0, command = selectLegCallback )
FRRadiobutton = Radiobutton( legSelectSubFrame, text = "FR", font = defaultFont, variable = rbLegVar,
                             value = 1, command = selectLegCallback )
RLRadiobutton = Radiobutton( legSelectSubFrame, text = "RL", font = defaultFont, variable = rbLegVar,
                             value = 2, command = selectLegCallback )
RRRadiobutton = Radiobutton( legSelectSubFrame, text = "RR", font = defaultFont, variable = rbLegVar,
                             value = 3, command = selectLegCallback )
FLRadiobutton.grid(row=1, column=0)
FRRadiobutton.grid(row=2, column=0)
RLRadiobutton.grid(row=3, column=0)
RRRadiobutton.grid(row=4, column=0)
FLRadiobutton.select()  # Set default


fkLabel = Label(jointSlidersFrame, text="FK - Joints", font = defaultFont)
fkLabel.grid(row=0, column=0)

jsLength = scsz*100
jsWidth = scsz*20

jsRange = 90.0
joint1Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j1",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint1SliderCallback )
joint1Slider.grid(row=1, column=0)

jsRange = 90.0
joint2Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j2",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint2SliderCallback )
joint2Slider.grid(row=2, column=0)

jsRange = 150.0
joint3Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j3",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint3SliderCallback )
joint3Slider.grid(row=3, column=0)

jsRange = 150.0
joint4Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j4",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint4SliderCallback )
joint4Slider.grid(row=4, column=0)

jsRange = 90.0
joint5Slider = Scale( jointSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j5",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint5SliderCallback )
joint5Slider.grid(row=5, column=0)


ikLabel = Label(targetSlidersFrame, text="IK - Target", font = defaultFont)
ikLabel.grid(row=0, column=0)

tsRange = 300.0
targetXSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "X",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetXSliderCallback )
targetXSlider.grid(row=1, column=0)

targetYSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Y",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetYSliderCallback )
targetYSlider.grid(row=2, column=0)

targetZSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Z",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetZSliderCallback )
targetZSlider.grid(row=3, column=0)

tsRange = 90.0
targetRollSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Roll",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetRollSliderCallback )
targetRollSlider.grid(row=4, column=0)

targetPitchSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Pitch",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetPitchSliderCallback )
targetPitchSlider.grid(row=5, column=0)

# Target Yaw has no effect
#targetYawSlider = Scale( targetSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Yaw",
#                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetYawSliderCallback )
#targetYawSlider.grid(row=6, column=0)
#targetYawSlider.config(state=DISABLED)


xyzLabel = Label(spineSlidersFrame, text="Spine - Trans.", font = defaultFont)
xyzLabel.grid(row=0, column=0)

tsRange = 300.0
spineXSlider = Scale( spineSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "X",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineXSliderCallback )
spineXSlider.grid(row=1, column=0)

spineYSlider = Scale( spineSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Y",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineYSliderCallback )
spineYSlider.grid(row=2, column=0)

spineZSlider = Scale( spineSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Z",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineZSliderCallback )
spineZSlider.grid(row=3, column=0)

rpyLabel = Label(spineSlidersFrame, text="Spine - Rot.", font = defaultFont)
rpyLabel.grid(row=0, column=1)

tsRange = 90.0
spineRollSlider = Scale( spineSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Roll",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineRollSliderCallback )
spineRollSlider.grid(row=1, column=1)

spinePitchSlider = Scale( spineSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Pitch",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spinePitchSliderCallback )
spinePitchSlider.grid(row=2, column=1)

spineYawSlider = Scale( spineSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Yaw",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineYawSliderCallback )
spineYawSlider.grid(row=3, column=1)

jsRange = 90.0
spineJoint1Slider = Scale( spineSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j1",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineJoint1SliderCallback )
spineJoint1Slider.grid(row=4, column=1)

spineJoint2Slider = Scale( spineSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j2",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineJoint2SliderCallback )
spineJoint2Slider.grid(row=5, column=1)


toggleIpVar = IntVar()
inputCheckButton = Checkbutton(buttonsFrame, text="Input", var=toggleIpVar, command=toggleInput, font = defaultFont)
inputCheckButton.grid(row=0, column=0)
#inputCheckButton.select()  # Set default

rbIpVar = IntVar()
kbInputRadioButton = Radiobutton( buttonsFrame, text="Keyboard", font = defaultFont, variable=rbIpVar,
                                  value = 0, command = selectInput )
jsInputRadioButton = Radiobutton( buttonsFrame, text="Joystick", font = defaultFont, variable=rbIpVar,
                                  value = 1, command = selectInput )
kbInputRadioButton.grid(row=0, column=1)
jsInputRadioButton.grid(row=0, column=2)
kbInputRadioButton.select()  # Set default

testIKButton = Button(buttonsFrame, text="Test IK", command=testIK, font = defaultFont)
testIKButton.grid(row=0, column=3)

loadTargets1Button = Button(buttonsFrame, text="Load 1", command=loadTargets1, font = defaultFont)
loadTargets1Button.grid(row=0, column=4)

loadTargets2Button = Button(buttonsFrame, text="Load 2", command=loadTargets2, font = defaultFont)
loadTargets2Button.grid(row=0, column=5)

quitButton = Button(buttonsFrame, text="Quit", command=quit, font = defaultFont)
quitButton.grid(row=0, column=6)


if __name__ == '__main__':
    global selectedLeg
    global selectedInput
    global a
    global spineAngleOffsets
    global legAngleOffsets
    global targetsHome, targets, speeds
    global showTargets
    initSpine()
    initLegs()
    initViews()
    selectedLeg = 0
    selectedInput = 0

    # Link lengths "a-1" (last value is the foot offset)
    a = [0, 29.05, 76.919, 72.96, 45.032, 33.596]

    # Offsets for natural "home" position
    spineAngleOffsets = [0, 0, -45]
    legAngleOffsets = [0, -34, 67.5, -33.5, 0]

    # Dummy targets (because of runSpikeIK, which calls runLegFK at the end)
    targets = [0, 0, 0, 0]
    for i, leg in enumerate(legs):
        targets[i] = identityTF()

    spine.angles = deepcopy(spineAngleOffsets)
    runSpineFK(spine, 0, 0, 0, 0, 0, 0)
    spineJoint1Slider.set(spine.angles[0])
    spineJoint2Slider.set(spine.angles[2])

    for leg in legs:
        leg.angles = deepcopy(legAngleOffsets)
        runLegFK(leg)
    joint1Slider.set(legs[selectedLeg].angles[0])
    joint2Slider.set(legs[selectedLeg].angles[1])
    joint3Slider.set(legs[selectedLeg].angles[2])
    joint4Slider.set(legs[selectedLeg].angles[3])
    joint5Slider.set(legs[selectedLeg].angles[4])

    # Targets: Foot in world
    targetsHome = [0, 0, 0, 0]
    speeds = [0, 0, 0, 0]
    for i, leg in enumerate(legs):
        targetsHome[i] = deepcopy(leg.joints[5].tfJointInWorld)
        applyYawPitchRoll( targetsHome[i], 0, 0, 0)
        speeds[i] = [0, 0, 0]
    targets = deepcopy(targetsHome)
    showTargets = True

    global inputModeSelect
    inputModeSelect = 0

    global inputLJSX, inputLJSY, inputRJSX, inputRJSY
    inputLJSX = 0
    inputLJSY = 0
    inputRJSX = 0
    inputRJSY = 0

    global inputKBX, inputKBY, inputKBZ
    inputKBX = 0
    inputKBY = 0
    inputKBZ = 0

    global inputForceMax, dragForceCoef
    inputForceMax = 1000
    dragForceCoef = 5

    gamepadReader = GamepadReader(root)
    gamepadReader.start()

    keyboardListener = KeyboardListener(root)
    keyboardListener.start()

    inputHandler = InputHandler(root)
    inputHandler.start()

    serialHandler = SerialHandler(root)
    serialHandler.start()

    App(root)
    root.mainloop()
