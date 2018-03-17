from HelperFunctions import rescale

import threading
import math
from copy import deepcopy
import inputs
from pynput import keyboard
from time import time, sleep


class KeyboardListener(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.terminate = False

        self.inputModeSelect = 0
        self.inputKBX1 = 0
        self.inputKBY1 = 0
        self.inputKBX2 = 0
        self.inputKBY2 = 0


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


    def stop(self):
        self.terminate = True
        self._Thread__stop()


    def run(self):
        # Collect events until released
        with keyboard.Listener(
                on_press = self.on_press,
                on_release = self.on_release) as listener:
            listener.join()


class GamepadReader(threading.Thread):
    def __init__(self, messageLogger):
        self.messageLogger = messageLogger

        threading.Thread.__init__(self)
        self.terminate = False
        self.gamepadOK = False
        self.gamepadUnplugged = False
        self.gamepadIOError = False

        self.inputModeSelect = 0
        self.inputLJSX = 0
        self.inputLJSY = 0
        self.inputRJSX = 0
        self.inputRJSY = 0


    def stop(self):
        self.terminate = True
        self._Thread__stop()


    def run(self):
        while not self.terminate:
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
                    self.messageLogger.log("Gamepad I/O error")
                    self.gamepadIOError = True
                sleep(1)
                continue


    def processEvent(self, event):
        #print(event.ev_type, event.code, event.state)
        if event.code == 'KEY_A':
            self.inputModeSelect = (self.inputModeSelect + 1) % 3
        elif event.code == 'ABS_X':
            self.inputLJSX = event.state
        elif event.code == 'ABS_Y':
            self.inputLJSY = event.state
        elif event.code == 'ABS_RX':
            self.inputRJSX = event.state
        elif event.code == 'ABS_RY':
            self.inputRJSY = event.state


class InputHandler(threading.Thread):
    def __init__(self, master, robot, keyboardListener, gamepadReader):
        self.master = master
        self.robot = robot
        self.keyboardListener = keyboardListener
        self.gamepadReader = gamepadReader

        # Threading vars
        threading.Thread.__init__(self)
        self.terminate = False
        self.paused = True
        self.triggerPolling = True
        self.cond = threading.Condition()

        # Input vars
        self.inputForceMax = 1000
        self.dragForceCoef = 5
        self.selectedInput = 0
        self.inputModeSelect = 0
        self.target = deepcopy(self.robot.targetsHome[self.robot.selectedLeg])
        self.speed = [0, 0, 0]
        self.inputX1Normed = 0
        self.inputY1Normed = 0
        self.inputX2Normed = 0
        self.inputY2Normed = 0
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

        if self.selectedInput == 0:
            # Keyboard
            self.inputX1Normed = self.filterInput(-self.keyboardListener.inputKBX1)
            self.inputY1Normed = self.filterInput(-self.keyboardListener.inputKBY1)
            self.inputX2Normed = self.filterInput(-self.keyboardListener.inputKBX2)
            self.inputY2Normed = self.filterInput(-self.keyboardListener.inputKBY2)
            self.inputModeSelect = self.keyboardListener.inputModeSelect
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
        with self.cond:
            if not self.paused:
                self.master.after(int(self.dt*1000), self.pollInputs)


    def pollIK(self):
        self.currTimeIK = time()
        #print "Poll IK time diff.", self.currTimeIK - self.prevTimeIK
        self.robot.targets[self.robot.selectedLeg] = deepcopy(self.target)
        self.robot.speeds[self.robot.selectedLeg] = deepcopy(self.speed)
        self.robot.runLegIK(self.robot.selectedLeg)
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
