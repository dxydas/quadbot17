#!/usr/bin/env python

import Globals
import CanvasDrawing
import InputControl
import SerialHandler
import RobotParts
from HelperFunctions import identityTF, applyYawPitchRoll
import Kinematics
import Gaits

from Tkinter import *
from time import time, localtime, strftime, sleep
import math
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
        canvasDrawing.redraw(Globals.targets, Globals.speeds, Globals.showTargets)
        self.prevTime = self.currTime
        self.master.after(int(self.dt*1000), self.poll)


def selectLegCallback():
    global selectedLeg
    Globals.selectedLeg = rbLegVar.get()


def joint1SliderCallback(val):
    robot.legs[Globals.selectedLeg].angles[0] = float(val)
    Kinematics.runLegFK(robot, Globals.selectedLeg)


def joint2SliderCallback(val):
    robot.legs[Globals.selectedLeg].angles[1] = float(val)
    Kinematics.runLegFK(robot, Globals.selectedLeg)


def joint3SliderCallback(val):
    robot.legs[Globals.selectedLeg].angles[2] = float(val)
    Kinematics.runLegFK(robot, Globals.selectedLeg)


def joint4SliderCallback(val):
    robot.legs[Globals.selectedLeg].angles[3] = float(val)
    Kinematics.runLegFK(robot, Globals.selectedLeg)


def joint5SliderCallback(val):
    robot.legs[Globals.selectedLeg].angles[4] = float(val)
    Kinematics.runLegFK(robot, Globals.selectedLeg)


def targetXSliderCallback(val):
    Globals.targets[Globals.selectedLeg][0, 3] = Globals.targetsHome[Globals.selectedLeg][0, 3] + float(val)
    Globals.targets[Globals.selectedLeg][1, 3] = Globals.targetsHome[Globals.selectedLeg][1, 3] + float(targetYSlider.get())
    Globals.targets[Globals.selectedLeg][2, 3] = Globals.targetsHome[Globals.selectedLeg][2, 3] + float(targetZSlider.get())
    Kinematics.runLegIK(robot, Globals.selectedLeg, Globals.targets[Globals.selectedLeg])


def targetYSliderCallback(val):
    Globals.targets[Globals.selectedLeg][0, 3] = Globals.targetsHome[Globals.selectedLeg][0, 3] + float(targetXSlider.get())
    Globals.targets[Globals.selectedLeg][1, 3] = Globals.targetsHome[Globals.selectedLeg][1, 3] + float(val)
    Globals.targets[Globals.selectedLeg][2, 3] = Globals.targetsHome[Globals.selectedLeg][2, 3] + float(targetZSlider.get())
    Kinematics.runLegIK(robot, Globals.selectedLeg, Globals.targets[Globals.selectedLeg])


def targetZSliderCallback(val):
    Globals.targets[Globals.selectedLeg][0, 3] = Globals.targetsHome[Globals.selectedLeg][0, 3] + float(targetXSlider.get())
    Globals.targets[Globals.selectedLeg][1, 3] = Globals.targetsHome[Globals.selectedLeg][1, 3] + float(targetYSlider.get())
    Globals.targets[Globals.selectedLeg][2, 3] = Globals.targetsHome[Globals.selectedLeg][2, 3] + float(val)
    Kinematics.runLegIK(robot, Globals.selectedLeg, Globals.targets[Globals.selectedLeg])


def targetRollSliderCallback(val):
    # Roll is rotation around X
    applyYawPitchRoll( Globals.targets[Globals.selectedLeg],
                       0,#targetYawSlider.get(),
                       targetPitchSlider.get(),
                       float(val))
    Kinematics.runLegIK(robot, Globals.selectedLeg, Globals.targets[Globals.selectedLeg])


def targetPitchSliderCallback(val):
    # Pitch is rotation around Y
    applyYawPitchRoll( Globals.targets[Globals.selectedLeg],
                       0,#targetYawSlider.get(),
                       float(val),
                       targetRollSlider.get() )
    Kinematics.runLegIK(robot, Globals.selectedLeg, Globals.targets[Globals.selectedLeg])


#def targetYawSliderCallback(val):
#    # Yaw is rotation around Z
#    applyYawPitchRoll( Globals.targets[Globals.selectedLeg],
#                       float(val),
#                       targetPitchSlider.get(),
#                       targetRollSlider.get() )
#    Kinematics.runLegIK(robot, Globals.selectedLeg, Globals.targets[Globals.selectedLeg])


def spineXSliderCallback(val):
    x = float(val)
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    Kinematics.runSpineFK(robot, Globals.targets, x, y, z, roll, pitch, yaw)


def spineYSliderCallback(val):
    x = spineXSlider.get()
    y = float(val)
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    Kinematics.runSpineFK(robot, Globals.targets, x, y, z, roll, pitch, yaw)


def spineZSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = float(val)
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    Kinematics.runSpineFK(robot, Globals.targets, x, y, z, roll, pitch, yaw)


def spineRollSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = float(val)
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    Kinematics.runSpineFK(robot, Globals.targets, x, y, z, roll, pitch, yaw)


def spinePitchSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = float(val)
    yaw = spineYawSlider.get()
    Kinematics.runSpineFK(robot, Globals.targets, x, y, z, roll, pitch, yaw)


def spineYawSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = float(val)
    Kinematics.runSpineFK(robot, Globals.targets, x, y, z, roll, pitch, yaw)


def spineJoint1SliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    robot.spine.angles[0] = float(val)
    Kinematics.runSpineFK(robot, Globals.targets, x, y, z, roll, pitch, yaw)
    # Dummy adjustment while IMU is not present:
    spineRollSlider.set( (Kinematics.spineAngleOffsets[0] + robot.spine.angles[0]) / 2.0 )
    spinePitchSlider.set( (Kinematics.spineAngleOffsets[2] - robot.spine.angles[2]) / 2.0 )


def spineJoint2SliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    robot.spine.angles[2] = float(val)
    Kinematics.runSpineFK(robot, Globals.targets, x, y, z, roll, pitch, yaw)
    # Dummy adjustment while IMU is not present
    spineRollSlider.set( (Kinematics.spineAngleOffsets[0] + robot.spine.angles[0]) / 2.0 )
    spinePitchSlider.set( (Kinematics.spineAngleOffsets[2] - robot.spine.angles[2]) / 2.0 )


def toggleInput():
    if toggleIpVar.get() == 0:
        inputHandler.pause()
    else:
        inputHandler.resume()


def selectInput():
    #global selectedInput
    Globals.selectedInput = rbIpVar.get()


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
        Globals.targets[Globals.selectedLeg][0, 3] = Globals.targetsHome[Globals.selectedLeg][0, 3] + x + xAdjust
        Globals.targets[Globals.selectedLeg][2, 3] = Globals.targetsHome[Globals.selectedLeg][2, 3] + y + yAdjust
        Kinematics.runLegIK(robot, Globals.selectedLeg, Globals.targets[Globals.selectedLeg])
        root.after(rateMsTIK, testIKCallback)


def quit():
    serialHandler.closeSerial()
    gamepadReader.stop()
    keyboardListener.stop()
    inputHandler.stop()
    serialHandler.stop()
    # Wait for threads to finish
    #print threading.active_count()
    while gamepadReader.isAlive() or inputHandler.isAlive() or serialHandler.isAlive():
        #print "waiting"
        sleep(0.1)
    #print threading.active_count()
    root.destroy()




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

# Scaling for 4K screens
if scsz == 2:
    root.tk.call('tk', 'scaling', 4.0)

defaultFont = ("System", 12)


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

Globals.messageBox = Text(messageBoxFrame, width = 32, height=18, font = defaultFont)
Globals.messageBox.grid(row=0, column=0, sticky=N+S+W+E)
scrl = Scrollbar(messageBoxFrame, command=Globals.messageBox.yview)
scrl.grid(row=0, column=1, sticky=N+S)
Globals.messageBox.config(yscrollcommand=scrl.set)
Globals.messageBox.bind("<<Modified>>", Globals.messageBoxModifiedCallback)
Globals.logMessage("Started at: " + startTime)


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

gaits = Gaits.Gaits(root)

loadTargets1Button = Button(buttonsFrame, text="Load 1", command=gaits.loadTargets1, font = defaultFont)
loadTargets1Button.grid(row=0, column=4)

loadTargets2Button = Button(buttonsFrame, text="Load 2", command=gaits.loadTargets2, font = defaultFont)
loadTargets2Button.grid(row=0, column=5)

quitButton = Button(buttonsFrame, text="Quit", command=quit, font = defaultFont)
quitButton.grid(row=0, column=6)


if __name__ == '__main__':
    robot = RobotParts.Robot()
    Globals.selectedLeg = 0
    Globals.selectedInput = 0

    # Dummy targets (because of runSpikeIK, which calls runLegFK at the end)
    Globals.targets = [0, 0, 0, 0]
    for i, leg in enumerate(robot.legs):
        Globals.targets[i] = identityTF()

    robot.spine.angles = deepcopy(Kinematics.spineAngleOffsets)
    Kinematics.runSpineFK(robot, Globals.targets, 0, 0, 0, 0, 0, 0)
    spineJoint1Slider.set(robot.spine.angles[0])
    spineJoint2Slider.set(robot.spine.angles[2])

    for i, leg in enumerate(robot.legs):
        leg.angles = deepcopy(Kinematics.legAngleOffsets)
        Kinematics.runLegFK(robot, i)
    joint1Slider.set(robot.legs[Globals.selectedLeg].angles[0])
    joint2Slider.set(robot.legs[Globals.selectedLeg].angles[1])
    joint3Slider.set(robot.legs[Globals.selectedLeg].angles[2])
    joint4Slider.set(robot.legs[Globals.selectedLeg].angles[3])
    joint5Slider.set(robot.legs[Globals.selectedLeg].angles[4])

    # Targets: Foot in world
    Globals.targetsHome = [0, 0, 0, 0]
    Globals.speeds = [0, 0, 0, 0]
    for i, leg in enumerate(robot.legs):
        Globals.targetsHome[i] = deepcopy(leg.joints[5].tfJointInWorld)
        applyYawPitchRoll( Globals.targetsHome[i], 0, 0, 0)
        Globals.speeds[i] = [0, 0, 0]
    Globals.targets = deepcopy(Globals.targetsHome)

    inputForceMax = 1000
    dragForceCoef = 5

    canvasDrawing = CanvasDrawing.CanvasDrawing(scsz, canvasW, canvasH, defaultFont,
                                                sideViewCanvas, frontViewCanvas, topViewCanvas,
                                                robot, inputForceMax)
    canvasDrawing.initViews()

    gamepadReader = InputControl.GamepadReader()
    gamepadReader.start()

    keyboardListener = InputControl.KeyboardListener()
    keyboardListener.start()

    inputHandler = InputControl.InputHandler(root, robot, inputForceMax, dragForceCoef)
    inputHandler.start()

    serialHandler = SerialHandler.SerialHandler(root)
    serialHandler.start()

    App(root)
    root.mainloop()
