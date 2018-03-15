import Globals
import Kinematics
from HelperFunctions import rescale

import threading
import math
from copy import deepcopy
import inputs
from pynput import keyboard
from time import time, sleep


inputModeSelect = 0

inputLJSX = 0
inputLJSY = 0
inputRJSX = 0
inputRJSY = 0

inputKBX = 0
inputKBY = 0
inputKBZ = 0


class GamepadReader(threading.Thread):
    def __init__(self):
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
                    Globals.logMessage("Gamepad connected")
                    self.gamepadOK = True
                    self.gamepadUnplugged = False
                except IndexError:
                    self.gamepadOK = False
                    if self.gamepadUnplugged == False:
                        Globals.logMessage("Gamepad not found")
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
                    Globals.logMessage("Gamepad I/O error")
                    self.gamepadIOError = True
                sleep(1)
                continue


    def processEvent(self, event):
        #print(event.ev_type, event.code, event.state)
        #global inputModeSelect, inputLJSX, inputLJSY, inputRJSX, inputRJSY
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
    def __init__(self):
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
    def __init__(self, master, robot, inputForceMax, dragForceCoef):
        self.master = master
        self.robot = robot
        self.inputForceMax = inputForceMax
        self.dragForceCoef = dragForceCoef
        # Threading vars
        threading.Thread.__init__(self)
        self.terminate = False
        self.paused = True
        self.triggerPolling = True
        self.cond = threading.Condition()
        # Input vars
        self.target = deepcopy(Globals.targetsHome[Globals.selectedLeg])
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
        if Globals.selectedInput == 0:
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
        Globals.targets[Globals.selectedLeg] = deepcopy(self.target)
        Globals.speeds[Globals.selectedLeg] = deepcopy(self.speed)
        Kinematics.runLegIK(self.robot, Globals.selectedLeg, Globals.targets[Globals.selectedLeg])
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
        F = self.inputForceMax*i - self.dragForceCoef*u0  # Force minus linear drag
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
