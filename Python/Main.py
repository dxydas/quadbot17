#!/usr/bin/env python


# Install Python, TKinter and modules:
# sudo apt-get install python python-pip python-tk
# sudo pip install numpy pynput inputs pyserial


import Robot
import CanvasDrawing
import InputControl
import SerialHandler
from HelperFunctions import identityTF, applyYawPitchRoll
import Gaits

from Tkinter import *
from time import time, localtime, strftime, sleep
import math
from copy import deepcopy


class App():
    def __init__(self, master):
        self.master = master
        self.dt = 0.01  # 10 ms
        self.prevTime = time()
        self.currTime = time()
        self.poll()  # Start polling

    def poll(self):
        self.currTime = time()
        #print "App time diff.", self.currTime - self.prevTime
        canvasDrawing.redraw()
        self.prevTime = self.currTime
        self.master.after(int(self.dt*1000), self.poll)


class MessageLogger():
    def __init__(self, messageBox):
        self.messageBox = messageBox
        self.messageBox.bind("<<Modified>>", self.messageBoxModifiedCallback)

    def log(self, msg):
        self.messageBox.insert(END, msg + "\n")

    def messageBoxModifiedCallback(self, event):
        self.messageBox.see(END)
        self.messageBox.edit_modified(False)


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
    robot.targets[robot.selectedLeg][0, 3] = robot.targetsHome[robot.selectedLeg][0, 3] + float(val)
    robot.targets[robot.selectedLeg][1, 3] = robot.targetsHome[robot.selectedLeg][1, 3] + float(targetYSlider.get())
    robot.targets[robot.selectedLeg][2, 3] = robot.targetsHome[robot.selectedLeg][2, 3] + float(targetZSlider.get())
    robot.runLegIK(robot.selectedLeg)


def targetYSliderCallback(val):
    robot.targets[robot.selectedLeg][0, 3] = robot.targetsHome[robot.selectedLeg][0, 3] + float(targetXSlider.get())
    robot.targets[robot.selectedLeg][1, 3] = robot.targetsHome[robot.selectedLeg][1, 3] + float(val)
    robot.targets[robot.selectedLeg][2, 3] = robot.targetsHome[robot.selectedLeg][2, 3] + float(targetZSlider.get())
    robot.runLegIK(robot.selectedLeg)


def targetZSliderCallback(val):
    robot.targets[robot.selectedLeg][0, 3] = robot.targetsHome[robot.selectedLeg][0, 3] + float(targetXSlider.get())
    robot.targets[robot.selectedLeg][1, 3] = robot.targetsHome[robot.selectedLeg][1, 3] + float(targetYSlider.get())
    robot.targets[robot.selectedLeg][2, 3] = robot.targetsHome[robot.selectedLeg][2, 3] + float(val)
    robot.runLegIK(robot.selectedLeg)


def targetRollSliderCallback(val):
    # Roll is rotation around X
    applyYawPitchRoll( robot.targets[robot.selectedLeg],
                       0,#targetYawSlider.get(),
                       targetPitchSlider.get(),
                       float(val))
    robot.runLegIK(robot.selectedLeg)


def targetPitchSliderCallback(val):
    # Pitch is rotation around Y
    applyYawPitchRoll( robot.targets[robot.selectedLeg],
                       0,#targetYawSlider.get(),
                       float(val),
                       targetRollSlider.get() )
    robot.runLegIK(robot.selectedLeg)


#def targetYawSliderCallback(val):
#    # Yaw is rotation around Z
#    applyYawPitchRoll( robot.targets[robot.selectedLeg],
#                       float(val),
#                       targetPitchSlider.get(),
#                       targetRollSlider.get() )
#    robot.runLegIK(robot.selectedLeg)


def spineXSliderCallback(val):
    x = float(val)
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    robot.runSpineFK(x, y, z, roll, pitch, yaw)


def spineYSliderCallback(val):
    x = spineXSlider.get()
    y = float(val)
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    robot.runSpineFK(x, y, z, roll, pitch, yaw)


def spineZSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = float(val)
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    robot.runSpineFK(x, y, z, roll, pitch, yaw)


def spineRollSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = float(val)
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    robot.runSpineFK(x, y, z, roll, pitch, yaw)


def spinePitchSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = float(val)
    yaw = spineYawSlider.get()
    robot.runSpineFK(x, y, z, roll, pitch, yaw)


def spineYawSliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = float(val)
    robot.runSpineFK(x, y, z, roll, pitch, yaw)


def spineJoint1SliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    robot.spine.angles[0] = float(val)
    robot.runSpineFK(x, y, z, roll, pitch, yaw)
    # Dummy adjustment while IMU is not present:
    spineRollSlider.set( (robot.spineAngleOffsets[0] + robot.spine.angles[0]) / 2.0 )
    spinePitchSlider.set( (robot.spineAngleOffsets[2] - robot.spine.angles[2]) / 2.0 )


def spineJoint2SliderCallback(val):
    x = spineXSlider.get()
    y = spineYSlider.get()
    z = spineZSlider.get()
    roll = spineRollSlider.get()
    pitch = spinePitchSlider.get()
    yaw = spineYawSlider.get()
    robot.spine.angles[2] = float(val)
    robot.runSpineFK(x, y, z, roll, pitch, yaw)
    # Dummy adjustment while IMU is not present
    spineRollSlider.set( (robot.spineAngleOffsets[0] + robot.spine.angles[0]) / 2.0 )
    spinePitchSlider.set( (robot.spineAngleOffsets[2] - robot.spine.angles[2]) / 2.0 )


def toggleInput():
    if toggleIpVar.get() == 0:
        inputHandler.pause()
    else:
        inputHandler.resume()


def selectInput():
    inputHandler.selectedInput = rbIpVar.get()


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




startTime = strftime("%a, %d %b %Y %H:%M:%S", localtime())

# Screen size var
# For HD screen, use 1
# For 4K screen, use 2
scsz = 2

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

messageBox = Text(messageBoxFrame, width = 32, height=18, font = defaultFont)
messageBox.grid(row=0, column=0, sticky=N+S+W+E)
scrl = Scrollbar(messageBoxFrame, command=messageBox.yview)
scrl.grid(row=0, column=1, sticky=N+S)
messageBox.config(yscrollcommand=scrl.set)

messageLogger = MessageLogger(messageBox)
messageLogger.log("Started at: " + startTime)


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

testIKButton = Button(buttonsFrame, text="Test IK", font = defaultFont)  # Callback gets defined later
testIKButton.grid(row=0, column=3)

loadTargets1Button = Button(buttonsFrame, text="Load 1", font = defaultFont)  # Callback gets defined later
loadTargets1Button.grid(row=0, column=4)

loadTargets2Button = Button(buttonsFrame, text="Load 2", font = defaultFont)  # Callback gets defined later
loadTargets2Button.grid(row=0, column=5)

quitButton = Button(buttonsFrame, text="Quit", command=quit, font = defaultFont)
quitButton.grid(row=0, column=6)




if __name__ == '__main__':
    robot = Robot.Robot(root)

    keyboardListener = InputControl.KeyboardListener()
    keyboardListener.start()

    gamepadReader = InputControl.GamepadReader(messageLogger)
    gamepadReader.start()

    inputHandler = InputControl.InputHandler(root, robot, keyboardListener, gamepadReader)
    inputHandler.start()

    serialHandler = SerialHandler.SerialHandler(root, messageLogger, robot)
    serialHandler.start()

    canvasDrawing = CanvasDrawing.CanvasDrawing(scsz, canvasW, canvasH, defaultFont,
                                                sideViewCanvas, frontViewCanvas, topViewCanvas,
                                                robot, inputHandler)
    canvasDrawing.initViews()

    gaits = Gaits.Gaits(root, robot, canvasDrawing)

    spineJoint1Slider.set(robot.spine.angles[0])
    spineJoint2Slider.set(robot.spine.angles[2])

    joint1Slider.set(robot.legs[robot.selectedLeg].angles[0])
    joint2Slider.set(robot.legs[robot.selectedLeg].angles[1])
    joint3Slider.set(robot.legs[robot.selectedLeg].angles[2])
    joint4Slider.set(robot.legs[robot.selectedLeg].angles[3])
    joint5Slider.set(robot.legs[robot.selectedLeg].angles[4])

    testIKButton.config(command=robot.testIK)
    loadTargets1Button.config(command=gaits.loadTargets1)
    loadTargets2Button.config(command=gaits.loadTargets1)

    App(root)
    root.mainloop()
