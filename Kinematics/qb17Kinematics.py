#!/usr/bin/env python

import sys
#import rospy
from std_msgs.msg import String
from Tkinter import *
#import tkMessageBox
#from geometry_msgs.msg import Vector3
#from std_msgs.msg import Float32
from time import localtime, strftime
import math
import numpy as np


def globalise():
    global sideViewCanvas, frontViewCanvas, topViewCanvas
    global canvasW, canvasH
    global canvasScale, canvasOffset
    global angles


class joint():
    def __init__(self, id, x, y, z):
        self.id = id
        self.x = x
        self.y = y
        self.z = z


def runFK(angles):

    a = [0, 0, 29.05, 76.919, 72.96, 45.032]  # Link lengths "a-1"

    angleOffsets = [0, 0, -34, 67.5, -33.5, 0]

    footOffset = 33.596

    s = [0, 0, 0, 0, 0, 0]
    c = [0, 0, 0, 0, 0, 0]
    for i in range(1,6):
        s[i] = math.sin( math.radians(angles[i-1] + angleOffsets[i]) )
        c[i] = math.cos( math.radians(angles[i-1] + angleOffsets[i]) )

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
    joint1 = joint( 1, T_1_in_W.item(0, 3), T_1_in_W.item(1, 3), T_1_in_W.item(2, 3) )
    T_2_in_W = T_1_in_W * T_2_in_1
    joint2 = joint( 2, T_2_in_W.item(0, 3), T_2_in_W.item(1, 3), T_2_in_W.item(2, 3) )
    T_3_in_W = T_2_in_W * T_3_in_2
    joint3 = joint( 3, T_3_in_W.item(0, 3), T_3_in_W.item(1, 3), T_3_in_W.item(2, 3) )
    T_4_in_W = T_3_in_W * T_4_in_3
    joint4 = joint( 4, T_4_in_W.item(0, 3), T_4_in_W.item(1, 3), T_4_in_W.item(2, 3) )
    T_5_in_W = T_4_in_W * T_5_in_4
    joint5 = joint( 5, T_5_in_W.item(0, 3), T_5_in_W.item(1, 3), T_5_in_W.item(2, 3) )
    T_F_in_W = T_5_in_W * T_F_in_5
    jointFoot = joint( 'F', T_F_in_W.item(0, 3), T_F_in_W.item(1, 3), T_F_in_W.item(2, 3) )

    # Redraw views
    sideViewCanvas.delete("clear")
    frontViewCanvas.delete("clear")
    topViewCanvas.delete("clear")

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


def messageBoxModifiedCallback(self):
    messageBox.see(END)
    messageBox.edit_modified(False)


def logMessage(msg):
    messageBox.insert(END, msg + "\n")


def quit():
    root.destroy()


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
topViewCanvas = Canvas(topViewFrame, background="#FFFACD", width = canvasW, height = canvasH)
topViewCanvas.grid(row=1, column=0, sticky=N+S+W+E)

messageBoxFrame = Frame(controlsFrame)
jointSlidersFrame = Frame(controlsFrame)
buttonsFrame = Frame(controlsFrame)

messageBoxFrame.grid(row=0, column=0)
jointSlidersFrame.grid(row=0, column=1)
buttonsFrame.grid(row=1, column=0)

messageBox = Text(messageBoxFrame, width = 42, height=18, font = defaultFont)
messageBox.grid(row=0, column=0, sticky=N+S+W+E)
scrl = Scrollbar(messageBoxFrame, command=messageBox.yview)
scrl.grid(row=0, column=1, sticky=N+S)
messageBox.config(yscrollcommand=scrl.set)
messageBox.bind("<<Modified>>", messageBoxModifiedCallback)
logMessage("Started at: " + startTime)

joint1Slider = Scale( jointSlidersFrame, from_ = -180.0, to = 180.0, resolution = 0.1, label = "j1",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint1SliderCallback )
joint1Slider.grid(row=0, column=0)

joint2Slider = Scale( jointSlidersFrame, from_ = -180.0, to = 180.0, resolution = 0.1, label = "j2",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint2SliderCallback )
joint2Slider.grid(row=1, column=0)

joint3Slider = Scale( jointSlidersFrame, from_ = -180.0, to = 180.0, resolution = 0.1, label = "j3",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint3SliderCallback )
joint3Slider.grid(row=2, column=0)

joint4Slider = Scale( jointSlidersFrame, from_ = -180.0, to = 180.0, resolution = 0.1, label = "j4",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint4SliderCallback )
joint4Slider.grid(row=3, column=0)

joint5Slider = Scale( jointSlidersFrame, from_ = -180.0, to = 180.0, resolution = 0.1, label = "j5",
                      length = 200, width = 40, font = 6, orient=HORIZONTAL, command = joint5SliderCallback )
joint5Slider.grid(row=4, column=0)

#testButton = Button(buttonsFrame, text="Test", command='', font = defaultFont)
#testButton.grid(row=0, column=0)

quitButton = Button(buttonsFrame, text="Quit", command=quit, font = defaultFont)
quitButton.grid(row=0, column=1)


if __name__ == '__main__':
    globalise()
    initViews()
    angles = [0, 0, 0, 0, 0]
    runFK(angles)
    mainloop()

