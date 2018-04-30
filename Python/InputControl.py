import Params
from HelperFunctions import rescale, getRollPitchYaw, applyYawPitchRoll

import threading
import math
from copy import deepcopy
import inputs
from pynput import keyboard
from time import time, sleep


class KeyboardReader():
    def __init__(self, messageLogger):
        self.messageLogger = messageLogger

        # Input vars
        self.numOfModes = 4
        self.inputKBX1 = 0
        self.inputKBY1 = 0
        self.inputKBX2 = 0
        self.inputKBY2 = 0

        self.listener = keyboard.Listener(on_press = self.on_press, on_release = self.on_release)
        self.listener.start()


    def on_press(self, key):
        if key == keyboard.Key.space:
            Params.inputModeSelect = (Params.inputModeSelect + 1) % Params.numOfModes
            self.messageLogger.log("Input mode changed - Mode: " + str(Params.inputModeSelect))
        elif key == keyboard.Key.left:
            self.inputKBX1 = -32768
        elif key == keyboard.Key.right:
            self.inputKBX1 = 32767
        elif key == keyboard.Key.up:
            self.inputKBY1 = -32768
        elif key == keyboard.Key.down:
            self.inputKBY1 = 32767
        elif key == keyboard.KeyCode.from_char('['):
            self.inputKBX2 = -32768
        elif key == keyboard.KeyCode.from_char(']'):
            self.inputKBX2 = 32767
        elif key == keyboard.Key.shift:
            self.inputKBY2 = -32768
        elif key == keyboard.Key.ctrl:
            self.inputKBY2 = 32767


    def on_release(self, key):
        if (key == keyboard.Key.right) or (key == keyboard.Key.left):
            self.inputKBX1 = 0
        elif (key == keyboard.Key.up) or (key == keyboard.Key.down):
            self.inputKBY1 = 0
        elif (key == keyboard.KeyCode.from_char('[')) or (key == keyboard.KeyCode.from_char(']')):
            self.inputKBX2 = 0
        elif (key == keyboard.Key.shift) or (key == keyboard.Key.ctrl):
            self.inputKBY2 = 0


    def stopListener(self):
        self.listener.stop()


class GamepadReader(threading.Thread):
    def __init__(self, messageLogger):
        self.messageLogger = messageLogger

        # Threading/timing vars
        threading.Thread.__init__(self)
        self.event = threading.Event()

        # Input vars
        self.gamepadOK = False
        self.gamepadUnplugged = False
        self.gamepadIOError = False
        self.inputLJSX = 0
        self.inputLJSY = 0
        self.inputRJSX = 0
        self.inputRJSY = 0


    def run(self):
        while not self.event.isSet():
            if not self.gamepadOK:
                self.devices = inputs.DeviceManager()
                try:
                    gamepad = self.devices.gamepads[0]
                    self.messageLogger.log("Gamepad connected")
                    self.gamepadOK = True
                    self.gamepadUnplugged = False
                except IndexError:
                    self.gamepadOK = False
                    if self.gamepadUnplugged == False:
                        self.messageLogger.log("Gamepad not found")
                        self.gamepadUnplugged = True
                    sleep(2)
            else:
                try:
                    # Get joystick input
                    gpEvents = gamepad.read()
                    for gpEvent in gpEvents:
                        self.processGamepadEvent(gpEvent)
                    self.gamepadIOError = False
                except IOError:
                    self.gamepadOK = False
                    if self.gamepadIOError == False:
                        self.messageLogger.log("Gamepad I/O error")
                        self.gamepadIOError = True
                    sleep(2)


    def stop(self):
        self.event.set()


    def processGamepadEvent(self, gpEvent):
        #print(gpEvent.ev_type, gpEvent.code, gpEvent.state)
        if gpEvent.code == 'BTN_SOUTH':  # Button A
            Params.inputModeSelect = (Params.inputModeSelect + 1) % Params.numOfModes
            self.messageLogger.log("Input mode changed - Mode: " + str(Params.inputModeSelect))
        elif gpEvent.code == 'ABS_X':
            self.inputLJSX = gpEvent.state
        elif gpEvent.code == 'ABS_Y':
            self.inputLJSY = gpEvent.state
        elif gpEvent.code == 'ABS_RX':
            self.inputRJSX = gpEvent.state
        elif gpEvent.code == 'ABS_RY':
            self.inputRJSY = gpEvent.state


class InputHandler(threading.Thread):
    def __init__(self, robot, keyboardReader, gamepadReader, messageLogger):
        self.robot = robot
        self.keyboardReader = keyboardReader
        self.gamepadReader = gamepadReader
        self.messageLogger = messageLogger

        # Threading/timing vars
        threading.Thread.__init__(self)
        self.event = threading.Event()
        self.dt = 0.05  # 50 ms
        self.paused = True
        self.prevTimeInputs = time()
        self.currTimeInputs = time()

        # Input vars
        self.selectedInput = 0
        self.legTarget = deepcopy(self.robot.legTargetsHome[self.robot.selectedLeg])
        self.legSpeed = [0, 0, 0]
        self.baseTarget = deepcopy(self.robot.baseTargetHome)
        self.spineSpeed = [0, 0, 0]
        roll, pitch, yaw = getRollPitchYaw(self.robot.baseTargetHome)
        self.spineRPY = [roll, pitch, yaw]
        self.spineRPYSpeed = [0, 0, 0]
        self.spineJoints = [robot.spineAngleOffsets[0], robot.spineAngleOffsets[2]]
        self.spineJointsSpeed = [0, 0]
        self.inputX1Normed = 0
        self.inputY1Normed = 0
        self.inputX2Normed = 0
        self.inputY2Normed = 0


    def run(self):
        while not self.event.isSet():
            if not self.paused:
                self.pollInputs()
                self.pollIK()
            self.event.wait(self.dt)


    def pause(self):
        self.paused = True


    def resume(self):
        self.paused = False


    def stop(self):
        self.event.set()


    def pollInputs(self):
        # Update current time
        self.currTimeInputs = time()

        if self.selectedInput == 0:
            # Keyboard
            self.inputX1Normed = self.filterInput(-self.keyboardReader.inputKBX1)
            self.inputY1Normed = self.filterInput(-self.keyboardReader.inputKBY1)
            self.inputX2Normed = self.filterInput(-self.keyboardReader.inputKBX2)
            self.inputY2Normed = self.filterInput(-self.keyboardReader.inputKBY2)
        else:
            # Joystick
            self.inputX1Normed = self.filterInput(-self.gamepadReader.inputLJSX)
            self.inputY1Normed = self.filterInput(-self.gamepadReader.inputLJSY)
            self.inputX2Normed = self.filterInput(-self.gamepadReader.inputRJSX)
            self.inputY2Normed = self.filterInput(-self.gamepadReader.inputRJSY)

        if Params.inputModeSelect == 0:
            # World X
            self.legTarget[0, 3], self.legSpeed[0] = self.updateMotion(self.inputY1Normed, self.legTarget[0, 3], self.legSpeed[0])
            # World Y
            self.legTarget[1, 3], self.legSpeed[1] = self.updateMotion(self.inputX1Normed, self.legTarget[1, 3], self.legSpeed[1])
            # World Z
            self.legTarget[2, 3], self.legSpeed[2] = self.updateMotion(self.inputY2Normed, self.legTarget[2, 3], self.legSpeed[2])
        elif Params.inputModeSelect == 1:
            # World X
            self.baseTarget[0, 3], self.spineSpeed[0] = self.updateMotion(self.inputY1Normed, self.baseTarget[0, 3], self.spineSpeed[0])
            # World Y
            self.baseTarget[1, 3], self.spineSpeed[1] = self.updateMotion(self.inputX1Normed, self.baseTarget[1, 3], self.spineSpeed[1])
            # World Z
            self.baseTarget[2, 3], self.spineSpeed[2] = self.updateMotion(self.inputY2Normed, self.baseTarget[2, 3], self.spineSpeed[2])
            # YPR
            applyYawPitchRoll(self.baseTarget, self.spineRPY[2], self.spineRPY[1], self.spineRPY[0])
        elif Params.inputModeSelect == 2:
            # World Roll
            self.spineRPY[0], self.spineRPYSpeed[0] = self.updateMotion(self.inputY1Normed, self.spineRPY[0], self.spineRPYSpeed[0])
            # World Pitch
            self.spineRPY[1], self.spineRPYSpeed[1] = self.updateMotion(self.inputX1Normed, self.spineRPY[1], self.spineRPYSpeed[1])
            # World Yaw
            self.spineRPY[2], self.spineRPYSpeed[2] = self.updateMotion(self.inputY2Normed, self.spineRPY[2], self.spineRPYSpeed[2])
            # YPR
            applyYawPitchRoll(self.baseTarget, self.spineRPY[2], self.spineRPY[1], self.spineRPY[0])
        elif Params.inputModeSelect == 3:
            # Front spine joint
            self.spineJoints[0], self.spineJointsSpeed[0] = self.updateMotion(self.inputY1Normed, self.spineJoints[0], self.spineJointsSpeed[0])
            # Rear spine joint
            self.spineJoints[1], self.spineJointsSpeed[1] = self.updateMotion(self.inputX1Normed, self.spineJoints[1], self.spineJointsSpeed[1])

        # Update previous time
        self.prevTimeInputs = self.currTimeInputs


    def pollIK(self):
        if Params.inputModeSelect == 0:
            self.robot.legTargets[self.robot.selectedLeg] = deepcopy(self.legTarget)
            self.robot.legTargetSpeeds[self.robot.selectedLeg] = deepcopy(self.legSpeed)
            self.robot.runLegIK(self.robot.selectedLeg)

        else:
            self.robot.spine.angles[0] = self.spineJoints[0]
            self.robot.spine.angles[2] = self.spineJoints[1]
            self.robot.baseTarget = deepcopy(self.baseTarget)
            self.robot.baseTargetSpeed = deepcopy(self.spineSpeed)
            self.robot.moveBase()


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
        m = 1.0
        u0 = speed
        # Force minus linear drag
        F = Params.inputForceMax*i - Params.dragForceCoef*u0
        a = F/m
        t = self.currTimeInputs - self.prevTimeInputs
        # Zero t if it's too large
        if t > 0.5:
            t = 0.0
        x0 = target
        # Equations of motion
        u = u0 + a*t
        x = x0 + u0*t + 0.5*a*math.pow(t, 2)
        return x, u
