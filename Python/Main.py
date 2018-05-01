#!/usr/bin/env python3


# Install Python, TKinter and modules:
# sudo apt-get install python python-pip python-tk
# sudo pip install numpy matplotlib pynput inputs pyserial


import Params
import Robot
import CanvasDrawing3D
import CanvasDrawing
import InputControl
import SerialHandler
from HelperFunctions import applyYawPitchRoll
import Gaits

import math
import threading
from time import localtime, strftime, sleep
import sys
if sys.version_info[0] < 3:
    from Tkinter import *
else:
    from tkinter import *


class MessageLogger():
    def __init__(self, messageBox):
        self.messageBox = messageBox
        self.messageBox.bind("<<Modified>>", self.messageBoxModifiedCallback)


    def log(self, msg):
        self.messageBox.insert(END, msg + "\n")


    def messageBoxModifiedCallback(self, event):
        self.messageBox.see(END)
        self.messageBox.edit_modified(False)


class TestIKTimer(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.event = threading.Event()


    def run(self):
        t = 2*math.pi
        while not self.event.isSet():
            if t >= 0:
                robot.testIKStep(t)
                t = t - 0.1
            else:
                self.stop()
            self.event.wait(0.05)


    def stop(self):
        self.event.set()


class LoadTargetsTimer(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.event = threading.Event()


    def run(self):
        t = 0
        if not Params.loadTargetsTimerRunning:
            while not self.event.isSet():
                if t < len(gaits.FLUpDown):
                    gaits.loadTargetsStep(t)
                    t = t + 1
                    Params.loadTargetsTimerRunning = True
                    Params.showTargets = False
                else:
                    Params.loadTargetsTimerRunning = False
                    Params.showTargets = True
                    self.stop()
                self.event.wait(0.05)
        else:
            t = gaits.findClosestLegPose()


    def stop(self):
        self.event.set()


def selectLegCallback():
    robot.selectedLeg = rbLegVar.get()


def joint1SliderCallback(val):
    robot.legs[robot.selectedLeg].angles[0] = float(val)
    robot.runLegFK(robot.selectedLeg)


def joint2SliderCallback(val):
    robot.legs[robot.selectedLeg].angles[1] = float(val)
    robot.runLegFK(robot.selectedLeg)


def joint3SliderCallback(val):
    robot.legs[robot.selectedLeg].angles[2] = float(val)
    robot.runLegFK(robot.selectedLeg)


def joint4SliderCallback(val):
    robot.legs[robot.selectedLeg].angles[3] = float(val)
    robot.runLegFK(robot.selectedLeg)


def joint5SliderCallback(val):
    robot.legs[robot.selectedLeg].angles[4] = float(val)
    robot.runLegFK(robot.selectedLeg)


def targetXSliderCallback(val):
    robot.legTargets[robot.selectedLeg][0, 3] = robot.legTargetsHome[robot.selectedLeg][0, 3] + float(val)
    robot.runLegIK(robot.selectedLeg)


def targetYSliderCallback(val):
    robot.legTargets[robot.selectedLeg][1, 3] = robot.legTargetsHome[robot.selectedLeg][1, 3] + float(val)
    robot.runLegIK(robot.selectedLeg)


def targetZSliderCallback(val):
    robot.legTargets[robot.selectedLeg][2, 3] = robot.legTargetsHome[robot.selectedLeg][2, 3] + float(val)
    robot.runLegIK(robot.selectedLeg)


def targetRollSliderCallback(val):
    # Roll is rotation around X
    applyYawPitchRoll( robot.legTargets[robot.selectedLeg],
                       0,#targetYawSlider.get(),
                       targetPitchSlider.get(),
                       float(val))
    robot.runLegIK(robot.selectedLeg)


def targetPitchSliderCallback(val):
    # Pitch is rotation around Y
    applyYawPitchRoll( robot.legTargets[robot.selectedLeg],
                       0,#targetYawSlider.get(),
                       float(val),
                       targetRollSlider.get() )
    robot.runLegIK(robot.selectedLeg)


#def targetYawSliderCallback(val):
#    # Yaw is rotation around Z
#    applyYawPitchRoll( robot.legTargets[robot.selectedLeg],
#                       float(val),
#                       targetPitchSlider.get(),
#                       targetRollSlider.get() )
#    robot.runLegIK(robot.selectedLeg)


def baseXSliderCallback(val):
    robot.baseTarget[0, 3] = robot.baseTargetHome[0, 3] + float(val)
    robot.moveBase()


def baseYSliderCallback(val):
    robot.baseTarget[1, 3] = robot.baseTargetHome[1, 3] + float(val)
    robot.moveBase()


def baseZSliderCallback(val):
    robot.baseTarget[2, 3] = robot.baseTargetHome[2, 3] + float(val)
    robot.moveBase()


def baseRollSliderCallback(val):
    roll = float(val)
    pitch = basePitchSlider.get()
    yaw = baseYawSlider.get()
    applyYawPitchRoll(robot.baseTarget, yaw, pitch, roll)
    robot.moveBase()


def basePitchSliderCallback(val):
    roll = baseRollSlider.get()
    pitch = float(val)
    yaw = baseYawSlider.get()
    applyYawPitchRoll(robot.baseTarget, yaw, pitch, roll)
    robot.moveBase()


def baseYawSliderCallback(val):
    roll = baseRollSlider.get()
    pitch = basePitchSlider.get()
    yaw = float(val)
    applyYawPitchRoll(robot.baseTarget, yaw, pitch, roll)
    robot.moveBase()


def spineJoint1SliderCallback(val):
    robot.spine.angles[0] = float(val)
    robot.moveBase()
    if Params.spineJointsDummyAdjustment:
        baseRollSlider.set( (robot.spineAngleOffsets[0] + robot.spine.angles[0]) / 2.0 )
        basePitchSlider.set( (robot.spineAngleOffsets[2] - robot.spine.angles[2]) / 2.0 )


def spineJoint2SliderCallback(val):
    robot.spine.angles[2] = float(val)
    robot.moveBase()
    if Params.spineJointsDummyAdjustment:
        baseRollSlider.set( (robot.spineAngleOffsets[0] + robot.spine.angles[0]) / 2.0 )
        basePitchSlider.set( (robot.spineAngleOffsets[2] - robot.spine.angles[2]) / 2.0 )


def toggleInput():
    if toggleIpVar.get() == 0:
        inputHandler.pause()
    else:
        inputHandler.resume()


def selectInput():
    inputHandler.selectedInput = rbIpVar.get()


def testIKCallback():
    testIKTimer = TestIKTimer()
    testIKTimer.start()


def loadTargets1Callback():
    loadTargets1Timer = LoadTargetsTimer()
    # Load from CSV
    gaits.loadFromFile("Gait_Creep.csv")
    loadTargets1Timer.start()


def loadTargets2Callback():
    loadTargets2Timer = LoadTargetsTimer()
    # Load from csv
    gaits.loadFromFile("Gait_Walk.csv")
    loadTargets2Timer.start()


def quit():
    keyboardReader.stopListener()
    gamepadReader.stop()
    inputHandler.stop()
    serialHandler.stop()
    serialHandler.closeSerial()
    root.destroy()




startTime = strftime("%a, %d %b %Y %H:%M:%S", localtime())

root = Tk()
root.title("Quadbot 17 Kinematics")
if Params.gui == 0:
    rootWidth = Params.scsz*860
    rootHeight = Params.scsz*370
elif Params.gui == 1:
    rootWidth = Params.scsz*1420
    rootHeight = Params.scsz*820
else:
    rootWidth = Params.scsz*1500#860
    rootHeight = Params.scsz*700#820
root.geometry("%dx%d" % (rootWidth, rootHeight))

# Scaling for 4K screens
if Params.scsz == 2:
    root.tk.call('tk', 'scaling', 4.0)


Grid.rowconfigure(root, 0, weight=1)
Grid.columnconfigure(root, 0, weight=1)


controlsFrame = Frame(root)

if Params.gui == 0:
    emptyFrame = Frame(root)
    emptyFrame.grid(row=0, column=0)
    controlsFrame.grid(row=1, column=0)

elif Params.gui == 1:
    sideViewFrame = Frame(root)
    topViewFrame = Frame(root)
    frontViewFrame = Frame(root)

    sideViewFrame.grid(row=0, column=0, sticky=N+W)
    frontViewFrame.grid(row=0, column=1, sticky=N+E)
    topViewFrame.grid(row=1, column=0, sticky=S+W)
    controlsFrame.grid(row=1, column=1, sticky=S+E)

    sideViewLabel = Label(sideViewFrame, text = "Side View", font = Params.defaultFont)
    sideViewLabel.grid(row=0, column=0)
    sideViewCanvas = Canvas(sideViewFrame, background = "#E0FFFF", width = Params.canvasW, height = Params.canvasH)
    sideViewCanvas.grid(row=1, column=0, sticky=N+S+W+E)

    frontViewLabel = Label(frontViewFrame, text = "Front View", font = Params.defaultFont)
    frontViewLabel.grid(row=0, column=0)
    frontViewCanvas = Canvas(frontViewFrame, background = "#FFFACD", width = Params.canvasW, height = Params.canvasH)
    frontViewCanvas.grid(row=1, column=0, sticky=N+S+W+E)

    topViewLabel = Label(topViewFrame, text = "Top View", font = Params.defaultFont)
    topViewLabel.grid(row=0, column=0)
    topViewCanvas = Canvas(topViewFrame, background = "#E0EEE0", width = Params.canvasW, height = Params.canvasH)
    topViewCanvas.grid(row=1, column=0, sticky=N+S+W+E)

else:
    canvasFrame = Frame(root)

    canvasFrame.grid(row=0, column=0)
    controlsFrame.grid(row=0, column=1)

    canvas = Canvas(canvasFrame, width = Params.canvasW, height = Params.canvasH)
    canvas.grid(row=0, column=0, sticky=N+S+W+E)


controlsSubFrame = Frame(controlsFrame)
buttonsFrame = Frame(controlsFrame)

messageBoxFrame = Frame(controlsSubFrame)
selectFrame = Frame(controlsSubFrame)
legJointsSlidersFrame = Frame(controlsSubFrame)
legTargetsSlidersFrame = Frame(controlsSubFrame)
baseMoveSlidersFrame = Frame(controlsSubFrame)
spineJointsSlidersFrame = Frame(controlsSubFrame)

messageBoxFrame.grid(row=0, column=0, sticky=N)
selectFrame.grid(row=0, column=1, sticky=N)
legJointsSlidersFrame.grid(row=0, column=2, sticky=N)
legTargetsSlidersFrame.grid(row=0, column=3, sticky=N)
baseMoveSlidersFrame.grid(row=0, column=4, sticky=N)
spineJointsSlidersFrame.grid(row=0, column=5, sticky=N)

legSelectSubFrame = Frame(selectFrame)
legSelectSubFrame.grid(row=0, column=0, sticky=N)

controlsSubFrame.grid(row=0, column=0, sticky=N)
buttonsFrame.grid(row=1, column=0, sticky=N)

messageBox = Text(messageBoxFrame, width = 32, height=18, font = Params.defaultFont)
messageBox.grid(row=0, column=0, sticky=N+S+W+E)
scrl = Scrollbar(messageBoxFrame, command=messageBox.yview)
scrl.grid(row=0, column=1, sticky=N+S)
messageBox.config(yscrollcommand=scrl.set)

messageLogger = MessageLogger(messageBox)
messageLogger.log("Started at: " + startTime)


legSelectLabel = Label(legSelectSubFrame, text = "Leg", font = Params.defaultFont)
legSelectLabel.grid(row=0, column=0)

rbLegVar = IntVar()
FLRadiobutton = Radiobutton( legSelectSubFrame, text = "FL", font = Params.defaultFont, variable = rbLegVar,
                             value = 0, command = selectLegCallback )
FRRadiobutton = Radiobutton( legSelectSubFrame, text = "FR", font = Params.defaultFont, variable = rbLegVar,
                             value = 1, command = selectLegCallback )
RLRadiobutton = Radiobutton( legSelectSubFrame, text = "RL", font = Params.defaultFont, variable = rbLegVar,
                             value = 2, command = selectLegCallback )
RRRadiobutton = Radiobutton( legSelectSubFrame, text = "RR", font = Params.defaultFont, variable = rbLegVar,
                             value = 3, command = selectLegCallback )
FLRadiobutton.grid(row=1, column=0)
FRRadiobutton.grid(row=2, column=0)
RLRadiobutton.grid(row=3, column=0)
RRRadiobutton.grid(row=4, column=0)
FLRadiobutton.select()  # Set default


legJointsLabel = Label(legJointsSlidersFrame, text = "Leg Joints", font = Params.defaultFont)
legJointsLabel.grid(row=0, column=0)

jsLength = Params.scsz*100
jsWidth = Params.scsz*20

jsRange = 90.0
joint1Slider = Scale( legJointsSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j1",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint1SliderCallback )
joint1Slider.grid(row=1, column=0)

jsRange = 90.0
joint2Slider = Scale( legJointsSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j2",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint2SliderCallback )
joint2Slider.grid(row=2, column=0)

jsRange = 150.0
joint3Slider = Scale( legJointsSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j3",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint3SliderCallback )
joint3Slider.grid(row=3, column=0)

jsRange = 150.0
joint4Slider = Scale( legJointsSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j4",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint4SliderCallback )
joint4Slider.grid(row=4, column=0)

jsRange = 90.0
joint5Slider = Scale( legJointsSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j5",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = joint5SliderCallback )
joint5Slider.grid(row=5, column=0)


legTargetsLabel = Label(legTargetsSlidersFrame, text = "Leg Targets", font = Params.defaultFont)
legTargetsLabel.grid(row=0, column=0)

tsRange = 300.0
targetXSlider = Scale( legTargetsSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "X",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetXSliderCallback )
targetXSlider.grid(row=1, column=0)

targetYSlider = Scale( legTargetsSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Y",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetYSliderCallback )
targetYSlider.grid(row=2, column=0)

targetZSlider = Scale( legTargetsSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Z",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetZSliderCallback )
targetZSlider.grid(row=3, column=0)

tsRange = 90.0
targetRollSlider = Scale( legTargetsSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Roll",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetRollSliderCallback )
targetRollSlider.grid(row=4, column=0)

targetPitchSlider = Scale( legTargetsSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Pitch",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetPitchSliderCallback )
targetPitchSlider.grid(row=5, column=0)

# Target Yaw has no effect
#targetYawSlider = Scale( legTargetsSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Yaw",
#                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = targetYawSliderCallback )
#targetYawSlider.grid(row=6, column=0)
#targetYawSlider.config(state=DISABLED)


moveLabel = Label(baseMoveSlidersFrame, text = "Base Move", font = Params.defaultFont)
moveLabel.grid(row=0, column=0)

tsRange = 300.0
baseXSlider = Scale( baseMoveSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "X",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = baseXSliderCallback )
baseXSlider.grid(row=1, column=0)

baseYSlider = Scale( baseMoveSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Y",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = baseYSliderCallback )
baseYSlider.grid(row=2, column=0)

baseZSlider = Scale( baseMoveSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 1.0, label = "Z",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = baseZSliderCallback )
baseZSlider.grid(row=3, column=0)

tsRange = 90.0
baseRollSlider = Scale( baseMoveSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Roll",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = baseRollSliderCallback )
baseRollSlider.grid(row=4, column=0)

basePitchSlider = Scale( baseMoveSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Pitch",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = basePitchSliderCallback )
basePitchSlider.grid(row=5, column=0)

baseYawSlider = Scale( baseMoveSlidersFrame, from_ = -tsRange, to = tsRange, resolution = 0.1, label = "Yaw",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = baseYawSliderCallback )
baseYawSlider.grid(row=6, column=0)

spineJointsLabel = Label(spineJointsSlidersFrame, text = "Spine Joints", font = Params.defaultFont)
spineJointsLabel.grid(row=0, column=0)

jsRange = 90.0
spineJoint1Slider = Scale( spineJointsSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j1",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineJoint1SliderCallback )
spineJoint1Slider.grid(row=1, column=0)

spineJoint2Slider = Scale( spineJointsSlidersFrame, from_ = -jsRange, to = jsRange, resolution = 0.1, label = "j2",
                      length = jsLength, width = jsWidth, font = ("System", 9), orient=HORIZONTAL, command = spineJoint2SliderCallback )
spineJoint2Slider.grid(row=2, column=0)


toggleIpVar = IntVar()
inputCheckButton = Checkbutton(buttonsFrame, text = "Input", var=toggleIpVar, command=toggleInput, font = Params.defaultFont)
inputCheckButton.grid(row=0, column=0, padx=10)
#inputCheckButton.select()  # Set default

rbIpVar = IntVar()
kbInputRadioButton = Radiobutton( buttonsFrame, text = "Keyboard", font = Params.defaultFont,
                                  variable=rbIpVar, value = 0, command = selectInput )
jsInputRadioButton = Radiobutton( buttonsFrame, text = "Joystick", font = Params.defaultFont,
                                  variable=rbIpVar, value = 1, command = selectInput )
kbInputRadioButton.grid(row=0, column=1, padx=10)
jsInputRadioButton.grid(row=0, column=2, padx=10)
kbInputRadioButton.select()  # Set default


testIKButton = Button(buttonsFrame, text = "Test IK", font = Params.defaultFont, command = testIKCallback)
testIKButton.grid(row=0, column=5)

loadTargets1Button = Button(buttonsFrame, text = "Load 1", font = Params.defaultFont, command = loadTargets1Callback)
loadTargets1Button.grid(row=0, column=6)

loadTargets2Button = Button(buttonsFrame, text = "Load 2", font = Params.defaultFont, command = loadTargets2Callback)
loadTargets2Button.grid(row=0, column=7)

quitButton = Button(buttonsFrame, text = "Quit", font = Params.defaultFont, command = quit)
quitButton.grid(row=0, column=8)




if __name__ == '__main__':
    robot = Robot.Robot()

    keyboardReader = InputControl.KeyboardReader(messageLogger)

    gamepadReader = InputControl.GamepadReader(messageLogger)
    gamepadReader.start()

    inputHandler = InputControl.InputHandler(robot, keyboardReader, gamepadReader, messageLogger)
    inputHandler.start()

    serialHandler = SerialHandler.SerialHandler(messageLogger, robot)
    serialHandler.start()

    if Params.gui == 0:
        pass
    elif Params.gui == 1:
        canvasDrawing = CanvasDrawing.CanvasDrawing(sideViewCanvas, frontViewCanvas, topViewCanvas, robot)
    else:
        canvasDrawing = CanvasDrawing3D.CanvasDrawing3D(canvas, robot)

    gaits = Gaits.Gaits(robot)

    spineJoint1Slider.set(robot.spine.angles[0])
    spineJoint2Slider.set(robot.spine.angles[2])

    joint1Slider.set(robot.legs[robot.selectedLeg].angles[0])
    joint2Slider.set(robot.legs[robot.selectedLeg].angles[1])
    joint3Slider.set(robot.legs[robot.selectedLeg].angles[2])
    joint4Slider.set(robot.legs[robot.selectedLeg].angles[3])
    joint5Slider.set(robot.legs[robot.selectedLeg].angles[4])

    root.mainloop()
