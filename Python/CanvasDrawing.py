import Params

import threading
import numpy as np
from time import time, sleep
import sys
if sys.version_info[0] < 3:
    from Tkconstants import NORMAL, HIDDEN
else:
    from tkinter.constants import NORMAL, HIDDEN


class CanvasDrawing():
    def __init__(self, sideViewCanvas, frontViewCanvas, topViewCanvas, robot):
        self.sideViewCanvas = sideViewCanvas
        self.frontViewCanvas = frontViewCanvas
        self.topViewCanvas = topViewCanvas
        self.robot = robot

        # Timing var
        self.dt = 0.05  # 50 ms

        # 1 mm -> scsz pixels
        self.canvasScale = Params.scsz

        # 3rd offset is for top view only
        self.canvasOffset = [-Params.canvasW/2, -Params.canvasH + Params.scsz*100, -Params.canvasH + Params.scsz*185]

        # Elements for each view
        v = 3
        self.allJointElements = [JointElements() for x in range(v)]
        self.allEndEffectorElements = [JointElements() for x in range(v)]
        self.allLinkElements = [LinkElements() for x in range(v)]
        self.allTargetElements = [TargetElements() for x in range(v)]

        self.initViews()

        self.redraw()


    def initViews(self):
        axisW = Params.scsz*2
        axisL = Params.scsz*30
        borderDist = Params.scsz*20

        # Side view axis widget
        self.sideViewCanvas.create_line( Params.canvasW - (borderDist + axisL), borderDist + axisL, Params.canvasW - borderDist, borderDist + axisL,
                                         fill = "red", width = axisW )  # x-axis
        self.sideViewCanvas.create_text( Params.canvasW - (borderDist + axisL), borderDist + axisL + Params.scsz*10, text = "X", fill = "red" )
        self.sideViewCanvas.create_line( Params.canvasW - borderDist, borderDist, Params.canvasW - borderDist, borderDist + axisL,
                                         fill = "blue", width = axisW )  # z-axis
        self.sideViewCanvas.create_text( Params.canvasW - borderDist + Params.scsz*10, borderDist, text = "Z", fill = "blue" )

        # Front view axis widget
        self.frontViewCanvas.create_line( Params.canvasW - (borderDist + axisL), borderDist + axisL, Params.canvasW - borderDist, borderDist + axisL,
                                          fill = "green", width = axisW )  # y-axis
        self.frontViewCanvas.create_text( Params.canvasW - borderDist, borderDist + axisL + Params.scsz*10, text = "Y", fill = "green" )
        self.frontViewCanvas.create_line( Params.canvasW - (borderDist + axisL), borderDist, Params.canvasW - (borderDist + axisL), borderDist + axisL,
                                          fill = "blue", width = axisW )  # z-axis
        self.frontViewCanvas.create_text( Params.canvasW - (borderDist + axisL) - Params.scsz*10, borderDist, text = "Z", fill = "blue" )

        # Top view axis widget
        self.topViewCanvas.create_line( Params.canvasW - (borderDist + axisL), borderDist, Params.canvasW - borderDist, borderDist,
                                        fill = "red", width = axisW )  # x-axis
        self.topViewCanvas.create_text( Params.canvasW - (borderDist + axisL), borderDist - Params.scsz*10, text = "X", fill = "red" )
        self.topViewCanvas.create_line( Params.canvasW - borderDist, borderDist, Params.canvasW - borderDist, borderDist + axisL,
                                        fill = "green", width = axisW )  # y-axis
        self.topViewCanvas.create_text( Params.canvasW - borderDist + Params.scsz*10, borderDist + axisL, text = "Y", fill = "green" )

        # Origin point on canvas
        r = Params.scsz*3
        fillCol = "black"
        borderCol = "black"
        w = Params.scsz*1
        self.sideViewCanvas.create_oval( Params.canvasW - r + self.canvasOffset[0], Params.canvasH - r + self.canvasOffset[1],
                                         Params.canvasW + r + self.canvasOffset[0], Params.canvasH + r + self.canvasOffset[1],
                                         fill = fillCol, outline = borderCol, width = w )
        self.frontViewCanvas.create_oval( Params.canvasW - r + self.canvasOffset[0], Params.canvasH - r + self.canvasOffset[1],
                                          Params.canvasW + r + self.canvasOffset[0], Params.canvasH + r + self.canvasOffset[1],
                                          fill = fillCol, outline = borderCol, width = w )
        self.topViewCanvas.create_oval( Params.canvasW - r + self.canvasOffset[0], Params.canvasH - r + self.canvasOffset[2],
                                        Params.canvasW + r + self.canvasOffset[0], Params.canvasH + r + self.canvasOffset[2],
                                        fill = fillCol, outline = borderCol, width = w )

        # Draw elements for the first time, store handles
        # Spine
        n = len(self.robot.spine.joints)
        for j in range(n-1, -1, -2):  # Skip dummy joint
            self.drawJoint( self.robot.spine.joints[j].id,
                            self.robot.spine.joints[j].tfJointInWorld[0, 3],
                            self.robot.spine.joints[j].tfJointInWorld[1, 3],
                            self.robot.spine.joints[j].tfJointInWorld[2, 3] )

        # Legs
        for leg in reversed(self.robot.legs):
            n = len(leg.joints)
            for j in range(n-2, -1, -1):
                self.drawLink( leg.joints[j].tfJointInWorld[0, 3],
                               leg.joints[j].tfJointInWorld[1, 3],
                               leg.joints[j].tfJointInWorld[2, 3],
                               leg.joints[j+1].tfJointInWorld[0, 3],
                               leg.joints[j+1].tfJointInWorld[1, 3],
                               leg.joints[j+1].tfJointInWorld[2, 3] )
            for j in range(n-2, -1, -1):
                self.drawJoint( leg.joints[j].id,
                                leg.joints[j].tfJointInWorld[0, 3],
                                leg.joints[j].tfJointInWorld[1, 3],
                                leg.joints[j].tfJointInWorld[2, 3] )
            self.drawEndEffector( leg.joints[n-1].id,
                                  leg.joints[n-1].tfJointInWorld[0, 3],
                                  leg.joints[n-1].tfJointInWorld[1, 3],
                                  leg.joints[n-1].tfJointInWorld[2, 3] )

        # Base target
        self.drawTarget(self.robot.baseTarget, self.robot.baseTargetSpeed)
        # Leg targets
        for i, target in enumerate(self.robot.legTargets):
            self.drawTarget(target, self.robot.legTargetSpeeds[i])


    def redraw(self):
        jointIdx = 0
        endEffectorIdx = 0
        linkIdx = 0
        targetIdx = 0
        # Spine
        n = len(self.robot.spine.joints)
        for j in range(n-1, -1, -2):  # Skip dummy joint
            self.moveJoint( jointIdx, self.robot.spine.joints[j].id,
                            self.robot.spine.joints[j].tfJointInWorld[0, 3],
                            self.robot.spine.joints[j].tfJointInWorld[1, 3],
                            self.robot.spine.joints[j].tfJointInWorld[2, 3] )
            jointIdx = jointIdx + 1

        # Legs
        for leg in reversed(self.robot.legs):
            n = len(leg.joints)
            for j in range(n-2, -1, -1):
                self.moveLink( linkIdx, leg.joints[j].tfJointInWorld[0, 3],
                               leg.joints[j].tfJointInWorld[1, 3],
                               leg.joints[j].tfJointInWorld[2, 3],
                               leg.joints[j+1].tfJointInWorld[0, 3],
                               leg.joints[j+1].tfJointInWorld[1, 3],
                               leg.joints[j+1].tfJointInWorld[2, 3] )
                linkIdx = linkIdx + 1
            for j in range(n-2, -1, -1):
                self.moveJoint( jointIdx, leg.joints[j].id,
                                leg.joints[j].tfJointInWorld[0, 3],
                                leg.joints[j].tfJointInWorld[1, 3],
                                leg.joints[j].tfJointInWorld[2, 3] )
                jointIdx = jointIdx + 1
            self.moveEndEffector( endEffectorIdx, leg.joints[n-1].id,
                                  leg.joints[n-1].tfJointInWorld[0, 3],
                                  leg.joints[n-1].tfJointInWorld[1, 3],
                                  leg.joints[n-1].tfJointInWorld[2, 3] )
            endEffectorIdx = endEffectorIdx + 1

        # Base target
        self.toggleTarget(targetIdx, Params.showTargets)
        if Params.showTargets:
            self.moveTarget(targetIdx, self.robot.baseTarget, self.robot.baseTargetSpeed)
        targetIdx = targetIdx + 1
        # Leg targets
        for i, target in enumerate(self.robot.legTargets):
            self.toggleTarget(targetIdx, Params.showTargets)
            if Params.showTargets:
                self.moveTarget(targetIdx, target, self.robot.legTargetSpeeds[i])
            targetIdx = targetIdx + 1

        self.sideViewCanvas.after(int(self.dt*1000), self.redraw)


    def drawJoint(self, id, x, y, z):
        r = Params.scsz*13
        fillCol = "#FFFFE0"
        borderCol = "#00008B"
        w = Params.scsz*3

        h1 = self.sideViewCanvas.create_oval(
                Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                fill = fillCol, outline = borderCol, width = w )
        h2 = self.sideViewCanvas.create_text(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                text = id, font = ("Times", 12, "bold") )
        self.allJointElements[0].circles.append(h1)
        self.allJointElements[0].texts.append(h2)

        h1 = self.frontViewCanvas.create_oval(
                Params.canvasW + self.canvasScale*y - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                Params.canvasW + self.canvasScale*y + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                fill = fillCol, outline = borderCol, width = w )
        h2 = self.frontViewCanvas.create_text(
                Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                text = id, font = ("Times", 12, "bold") )
        self.allJointElements[1].circles.append(h1)
        self.allJointElements[1].texts.append(h2)

        h1 = self.topViewCanvas.create_oval(
                Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
                Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + r + self.canvasOffset[2],
                fill = fillCol, outline = borderCol, width = w )
        h2 = self.topViewCanvas.create_text(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
                text = id, font = ("Times", 12, "bold") )
        self.allJointElements[2].circles.append(h1)
        self.allJointElements[2].texts.append(h2)


    def drawEndEffector(self, id, x, y, z):
        r = Params.scsz*13
        fillCol = "#00008B"
        borderCol = "#00008B"
        w = Params.scsz*3
        h1 = self.sideViewCanvas.create_oval(
                Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                fill = fillCol, outline = borderCol, width = w )
        h2 = self.sideViewCanvas.create_text(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                text = id, fill = "white", font = ("Times", 12, "bold") )
        self.allEndEffectorElements[0].circles.append(h1)
        self.allEndEffectorElements[0].texts.append(h2)

        h1 = self.frontViewCanvas.create_oval(
                Params.canvasW + self.canvasScale*y - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                Params.canvasW + self.canvasScale*y + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                fill = fillCol, outline = borderCol, width = w )
        h2 = self.frontViewCanvas.create_text(
                Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                text = id, fill = "white", font = ("Times", 12, "bold") )
        self.allEndEffectorElements[1].circles.append(h1)
        self.allEndEffectorElements[1].texts.append(h2)

        h1 = self.topViewCanvas.create_oval(
                Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
                Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + r + self.canvasOffset[2],
                fill = fillCol, outline = borderCol, width = w )
        h2 = self.topViewCanvas.create_text(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
                text = id, fill = "white", font = ("Times", 12, "bold") )
        self.allEndEffectorElements[2].circles.append(h1)
        self.allEndEffectorElements[2].texts.append(h2)


    def drawLink(self, Ax, Ay, Az, Bx, By, Bz):
        fillCol = "#00008B"
        w = Params.scsz*5
        h = self.sideViewCanvas.create_line(
                Params.canvasW - self.canvasScale*Ax + self.canvasOffset[0], Params.canvasH - self.canvasScale*Az + self.canvasOffset[1],
                Params.canvasW - self.canvasScale*Bx + self.canvasOffset[0], Params.canvasH - self.canvasScale*Bz + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allLinkElements[0].lines.append(h)

        h = self.frontViewCanvas.create_line(
                Params.canvasW + self.canvasScale*Ay + self.canvasOffset[0], Params.canvasH - self.canvasScale*Az + self.canvasOffset[1],
                Params.canvasW + self.canvasScale*By + self.canvasOffset[0], Params.canvasH - self.canvasScale*Bz + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allLinkElements[1].lines.append(h)

        h = self.topViewCanvas.create_line(
                Params.canvasW - self.canvasScale*Ax + self.canvasOffset[0], Params.canvasH + self.canvasScale*Ay + self.canvasOffset[2],
                Params.canvasW - self.canvasScale*Bx + self.canvasOffset[0], Params.canvasH + self.canvasScale*By + self.canvasOffset[2],
                fill = fillCol, width = w )
        self.allLinkElements[2].lines.append(h)


    def drawTarget(self, target, speed):
        # Target circle
        r = Params.scsz*16
        borderCol = "green"
        w = Params.scsz*5
        x = target[0, 3]
        y = target[1, 3]
        z = target[2, 3]
        h = self.sideViewCanvas.create_oval(
                Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                outline = borderCol, width = w )
        self.allTargetElements[0].circles.append(h)

        h = self.frontViewCanvas.create_oval(
                Params.canvasW + self.canvasScale*y - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                Params.canvasW + self.canvasScale*y + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                outline = borderCol, width = w )
        self.allTargetElements[1].circles.append(h)

        h = self.topViewCanvas.create_oval(
                Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
                Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + r + self.canvasOffset[2],
                outline = borderCol, width = w )
        self.allTargetElements[2].circles.append(h)

        # Line along X
        tmpVec = np.array([50, 0, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        fillCol = "red"
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        h = self.sideViewCanvas.create_line(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allTargetElements[0].frameXLines.append(h)

        h = self.frontViewCanvas.create_line(
                Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                Params.canvasW + self.canvasScale*ly + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allTargetElements[1].frameXLines.append(h)

        h = self.topViewCanvas.create_line(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
                Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH + self.canvasScale*ly + self.canvasOffset[2],
                fill = fillCol, width = w )
        self.allTargetElements[2].frameXLines.append(h)

        # Line along Y
        tmpVec = np.array([0, 50, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        fillCol = "green"
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        h = self.sideViewCanvas.create_line(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allTargetElements[0].frameYLines.append(h)

        h = self.frontViewCanvas.create_line(
                Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                Params.canvasW + self.canvasScale*ly + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allTargetElements[1].frameYLines.append(h)

        h = self.topViewCanvas.create_line(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
                Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH + self.canvasScale*ly + self.canvasOffset[2],
                fill = fillCol, width = w )
        self.allTargetElements[2].frameYLines.append(h)

        # Line along Z
        tmpVec = np.array([0, 0, 50, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        fillCol = "blue"
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        h = self.sideViewCanvas.create_line(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allTargetElements[0].frameZLines.append(h)

        h = self.frontViewCanvas.create_line(
                Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                Params.canvasW + self.canvasScale*ly + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allTargetElements[1].frameZLines.append(h)

        h = self.topViewCanvas.create_line(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
                Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH + self.canvasScale*ly + self.canvasOffset[2],
                fill = fillCol, width = w )
        self.allTargetElements[2].frameZLines.append(h)

        # Speed vector
        fillCol = "#39FF14"
        sx = speed[0]
        sy = speed[1]
        sz = speed[2]
        k = 400.0 / Params.inputForceMax  # Arbitrary scaling, to make max. length of vector constant
        h = self.sideViewCanvas.create_line(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                Params.canvasW - self.canvasScale*(x + sx*k) + self.canvasOffset[0], Params.canvasH - self.canvasScale*(z + sz*k) + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allTargetElements[0].speedLines.append(h)

        h = self.frontViewCanvas.create_line(
                Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
                Params.canvasW + self.canvasScale*(y + sy*k) + self.canvasOffset[0], Params.canvasH - self.canvasScale*(z + sz*k) + self.canvasOffset[1],
                fill = fillCol, width = w )
        self.allTargetElements[1].speedLines.append(h)

        h = self.topViewCanvas.create_line(
                Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
                Params.canvasW - self.canvasScale*(x + sx*k) + self.canvasOffset[0], Params.canvasH + self.canvasScale*(y + sy*k) + self.canvasOffset[2],
                fill = fillCol, width = w )
        self.allTargetElements[2].speedLines.append(h)


    def moveJoint(self, index, id, x, y, z):
        r = Params.scsz*13
        self.sideViewCanvas.coords(
            self.allJointElements[0].circles[index],
            Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
            Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1] )
        self.sideViewCanvas.coords(
            self.allJointElements[0].texts[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1] )

        self.frontViewCanvas.coords(
            self.allJointElements[1].circles[index],
            Params.canvasW + self.canvasScale*y - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
            Params.canvasW + self.canvasScale*y + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1] )
        self.frontViewCanvas.coords(
            self.allJointElements[1].texts[index],
            Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1] )

        self.topViewCanvas.coords(
            self.allJointElements[2].circles[index],
            Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
            Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + r + self.canvasOffset[2] )
        self.topViewCanvas.coords(
            self.allJointElements[2].texts[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2] )


    def moveEndEffector(self, index, id, x, y, z):
        r = Params.scsz*13
        self.sideViewCanvas.coords (
            self.allEndEffectorElements[0].circles[index],
            Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
            Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1] )
        self.sideViewCanvas.coords(
            self.allEndEffectorElements[0].texts[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1] )

        self.frontViewCanvas.coords(
            self.allEndEffectorElements[1].circles[index],
            Params.canvasW + self.canvasScale*y - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
            Params.canvasW + self.canvasScale*y + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1] )
        self.frontViewCanvas.coords(
            self.allEndEffectorElements[1].texts[index],
            Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1] )

        self.topViewCanvas.coords(
            self.allEndEffectorElements[2].circles[index],
            Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
            Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + r + self.canvasOffset[2] )
        self.topViewCanvas.coords(
            self.allEndEffectorElements[2].texts[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2] )


    def moveLink(self, index, Ax, Ay, Az, Bx, By, Bz):
        self.sideViewCanvas.coords(
            self.allLinkElements[0].lines[index],
            Params.canvasW - self.canvasScale*Ax + self.canvasOffset[0], Params.canvasH - self.canvasScale*Az + self.canvasOffset[1],
            Params.canvasW - self.canvasScale*Bx + self.canvasOffset[0], Params.canvasH - self.canvasScale*Bz + self.canvasOffset[1] )

        self.frontViewCanvas.coords(
            self.allLinkElements[1].lines[index],
            Params.canvasW + self.canvasScale*Ay + self.canvasOffset[0], Params.canvasH - self.canvasScale*Az + self.canvasOffset[1],
            Params.canvasW + self.canvasScale*By + self.canvasOffset[0], Params.canvasH - self.canvasScale*Bz + self.canvasOffset[1] )

        self.topViewCanvas.coords(
            self.allLinkElements[2].lines[index],
            Params.canvasW - self.canvasScale*Ax + self.canvasOffset[0], Params.canvasH + self.canvasScale*Ay + self.canvasOffset[2],
            Params.canvasW - self.canvasScale*Bx + self.canvasOffset[0], Params.canvasH + self.canvasScale*By + self.canvasOffset[2] )


    def moveTarget(self, index, target, speed):
        # Target circle
        r = Params.scsz*16
        x = target[0, 3]
        y = target[1, 3]
        z = target[2, 3]
        self.sideViewCanvas.coords(
            self.allTargetElements[0].circles[index],
            Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
            Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1] )

        self.frontViewCanvas.coords(
            self.allTargetElements[1].circles[index],
            Params.canvasW + self.canvasScale*y - r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
            Params.canvasW + self.canvasScale*y + r + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + r + self.canvasOffset[1] )

        self.topViewCanvas.coords(
            self.allTargetElements[2].circles[index],
            Params.canvasW - self.canvasScale*x - r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
            Params.canvasW - self.canvasScale*x + r + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + r + self.canvasOffset[2] )

        # Line along X
        tmpVec = np.array([50, 0, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.sideViewCanvas.coords(
            self.allTargetElements[0].frameXLines[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
            Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1] )

        self.frontViewCanvas.coords(
            self.allTargetElements[1].frameXLines[index],
            Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
            Params.canvasW + self.canvasScale*ly + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1] )

        self.topViewCanvas.coords(
            self.allTargetElements[2].frameXLines[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
            Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH + self.canvasScale*ly + self.canvasOffset[2] )

        # Line along Y
        tmpVec = np.array([0, 50, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.sideViewCanvas.coords(
            self.allTargetElements[0].frameYLines[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
            Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1] )

        self.frontViewCanvas.coords(
            self.allTargetElements[1].frameYLines[index],
            Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
            Params.canvasW + self.canvasScale*ly + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1] )

        self.topViewCanvas.coords(
            self.allTargetElements[2].frameYLines[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
            Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH + self.canvasScale*ly + self.canvasOffset[2] )

        # Line along Z
        tmpVec = np.array([0, 0, 50, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.sideViewCanvas.coords(
            self.allTargetElements[0].frameZLines[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
            Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1] )

        self.frontViewCanvas.coords(
            self.allTargetElements[1].frameZLines[index],
            Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
            Params.canvasW + self.canvasScale*ly + self.canvasOffset[0], Params.canvasH - self.canvasScale*lz + self.canvasOffset[1] )

        self.topViewCanvas.coords(
            self.allTargetElements[2].frameZLines[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
            Params.canvasW - self.canvasScale*lx + self.canvasOffset[0], Params.canvasH + self.canvasScale*ly + self.canvasOffset[2] )

        # Speed vector
        sx = speed[0]
        sy = speed[1]
        sz = speed[2]
        k = 400.0 / Params.inputForceMax  # Arbitrary scaling, to make max. length of vector constant
        self.sideViewCanvas.coords(
            self.allTargetElements[0].speedLines[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
            Params.canvasW - self.canvasScale*(x + sx*k) + self.canvasOffset[0], Params.canvasH - self.canvasScale*(z + sz*k) + self.canvasOffset[1] )

        self.frontViewCanvas.coords(
            self.allTargetElements[1].speedLines[index],
            Params.canvasW + self.canvasScale*y + self.canvasOffset[0], Params.canvasH - self.canvasScale*z + self.canvasOffset[1],
            Params.canvasW + self.canvasScale*(y + sy*k) + self.canvasOffset[0], Params.canvasH - self.canvasScale*(z + sz*k) + self.canvasOffset[1] )

        self.topViewCanvas.coords(
            self.allTargetElements[2].speedLines[index],
            Params.canvasW - self.canvasScale*x + self.canvasOffset[0], Params.canvasH + self.canvasScale*y + self.canvasOffset[2],
            Params.canvasW - self.canvasScale*(x + sx*k) + self.canvasOffset[0], Params.canvasH + self.canvasScale*(y + sy*k) + self.canvasOffset[2] )


    def toggleTarget(self, index, show):
        if show:
            s = NORMAL
        else:
            s = HIDDEN
        for x in range(3):
            self.sideViewCanvas.itemconfig(self.allTargetElements[x].circles[index], state=s)
            self.sideViewCanvas.itemconfig(self.allTargetElements[x].frameXLines[index], state=s)
            self.sideViewCanvas.itemconfig(self.allTargetElements[x].frameYLines[index], state=s)
            self.sideViewCanvas.itemconfig(self.allTargetElements[x].frameZLines[index], state=s)
            self.sideViewCanvas.itemconfig(self.allTargetElements[x].speedLines[index], state=s)

            self.frontViewCanvas.itemconfig(self.allTargetElements[x].circles[index], state=s)
            self.frontViewCanvas.itemconfig(self.allTargetElements[x].frameXLines[index], state=s)
            self.frontViewCanvas.itemconfig(self.allTargetElements[x].frameYLines[index], state=s)
            self.frontViewCanvas.itemconfig(self.allTargetElements[x].frameZLines[index], state=s)
            self.frontViewCanvas.itemconfig(self.allTargetElements[x].speedLines[index], state=s)

            self.topViewCanvas.itemconfig(self.allTargetElements[x].circles[index], state=s)
            self.topViewCanvas.itemconfig(self.allTargetElements[x].frameXLines[index], state=s)
            self.topViewCanvas.itemconfig(self.allTargetElements[x].frameYLines[index], state=s)
            self.topViewCanvas.itemconfig(self.allTargetElements[x].frameZLines[index], state=s)
            self.topViewCanvas.itemconfig(self.allTargetElements[x].speedLines[index], state=s)


class JointElements():
    def __init__(self):
        self.circles = []
        self.texts = []


class LinkElements():
    def __init__(self):
        self.lines = []


class TargetElements():
    def __init__(self):
        self.circles = []
        self.frameXLines = []
        self.frameYLines = []
        self.frameZLines = []
        self.speedLines = []
