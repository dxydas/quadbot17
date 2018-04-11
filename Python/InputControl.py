import Globals
from HelperFunctions import rescale

import threading
import math
from copy import deepcopy
import inputs
from pynput import keyboard
from time import time, sleep


class KeyboardReader():
    def __init__(self):
        self.inputModeSelect = 0
        self.inputKBX1 = 0
        self.inputKBY1 = 0
        self.inputKBX2 = 0
        self.inputKBY2 = 0

        self.listener = keyboard.Listener(on_press = self.on_press, on_release = self.on_release)
        self.listener.start()


    def on_press(self, key):
        if key == keyboard.Key.space:
            self.inputModeSelect = (self.inputModeSelect + 1) % 3
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
        self.dt = 0.05  # 50 ms

        # Input vars
        self.gamepadOK = False
        self.gamepadUnplugged = False
        self.gamepadIOError = False
        self.inputModeSelect = 0
        self.inputLJSX = 0
        self.inputLJSY = 0
        self.inputRJSX = 0
        self.inputRJSY = 0
        self.prevTime = time()
        self.currTime = time()


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
                    #print("GamepadReader poll gamepad time diff.:", self.currTime - self.prevTime)
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
            self.event.wait(self.dt)


    def stop(self):
        self.event.set()


    def processGamepadEvent(self, gpEvent):
        self.currTime = time()
        #print(gpEvent.ev_type, gpEvent.code, gpEvent.state)
        if gpEvent.code == 'BTN_SOUTH':  # Button A
            self.inputModeSelect = (self.inputModeSelect + 1) % 3
        elif gpEvent.code == 'ABS_X':
            self.inputLJSX = gpEvent.state
        elif gpEvent.code == 'ABS_Y':
            self.inputLJSY = gpEvent.state
        elif gpEvent.code == 'ABS_RX':
            self.inputRJSX = gpEvent.state
        elif gpEvent.code == 'ABS_RY':
            self.inputRJSY = gpEvent.state
        self.prevTime = self.currTime


class InputHandler(threading.Thread):
    def __init__(self, robot, keyboardReader, gamepadReader):
        self.robot = robot
        self.keyboardReader = keyboardReader
        self.gamepadReader = gamepadReader

        # Threading/timing vars
        threading.Thread.__init__(self)
        self.event = threading.Event()
        self.dt = 0.05  # 50 ms
        self.paused = True

        # Input vars
        self.selectedInput = 0
        self.inputModeSelect = 0
        self.target = deepcopy(self.robot.targetsHome[self.robot.selectedLeg])
        self.speed = [0, 0, 0]
        self.inputX1Normed = 0
        self.inputY1Normed = 0
        self.inputX2Normed = 0
        self.inputY2Normed = 0
        self.prevTimeInputs = time()
        self.currTimeInputs = time()
        self.prevTimeIK = time()
        self.currTimeIK = time()


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
        self.currTimeInputs = time()
        if self.selectedInput == 0:
            # Keyboard
            self.inputX1Normed = self.filterInput(-self.keyboardReader.inputKBX1)
            self.inputY1Normed = self.filterInput(-self.keyboardReader.inputKBY1)
            self.inputX2Normed = self.filterInput(-self.keyboardReader.inputKBX2)
            self.inputY2Normed = self.filterInput(-self.keyboardReader.inputKBY2)
            self.inputModeSelect = self.keyboardReader.inputModeSelect
        else:
            # Joystick
            self.inputX1Normed = self.filterInput(-self.gamepadReader.inputLJSX)
            self.inputY1Normed = self.filterInput(-self.gamepadReader.inputLJSY)
            self.inputX2Normed = self.filterInput(-self.gamepadReader.inputRJSX)
            self.inputY2Normed = self.filterInput(-self.gamepadReader.inputRJSY)
            self.inputModeSelect = self.gamepadReader.inputModeSelect

        if self.inputModeSelect == 0:
            # World X
            self.target[0, 3], self.speed[0] = self.updateMotion(self.inputY1Normed, self.target[0, 3], self.speed[0])
            # World Y
            self.target[1, 3], self.speed[1] = self.updateMotion(self.inputX1Normed, self.target[1, 3], self.speed[1])
            # World Z
            self.target[2, 3], self.speed[2] = self.updateMotion(self.inputY2Normed, self.target[2, 3], self.speed[2])
        elif self.inputModeSelect == 1:
            pass
        elif self.inputModeSelect == 2:
            pass
        self.prevTimeInputs = self.currTimeInputs


    def pollIK(self):
        self.currTimeIK = time()
        self.robot.targets[self.robot.selectedLeg] = deepcopy(self.target)
        self.robot.speeds[self.robot.selectedLeg] = deepcopy(self.speed)
        self.robot.runLegIK(self.robot.selectedLeg)
        self.prevTimeIK = self.currTimeIK


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
        F = Globals.inputForceMax*i - Globals.dragForceCoef*u0
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
