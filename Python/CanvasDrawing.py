import numpy as np


class CanvasDrawing():
    def __init__(self, scsz, canvasW, canvasH, defaultFont, sideViewCanvas, frontViewCanvas, topViewCanvas, robot, inputHandler):
        self.scsz = scsz
        self.canvasW = canvasW
        self.canvasH = canvasH
        self.defaultFont = defaultFont
        self.sideViewCanvas = sideViewCanvas
        self.frontViewCanvas = frontViewCanvas
        self.topViewCanvas = topViewCanvas
        self.robot = robot
        self.inputHandler = inputHandler

        # 1 mm -> scsz pixels
        self.canvasScale = self.scsz 

        # 3rd offset is for top view only
        self.canvasOffset = [-self.canvasW/2, -self.canvasH + self.scsz*100, -self.canvasH + self.scsz*185]

        self.showTargets = True

        self.initViews()


    def initViews(self):
        axisW = self.scsz*2
        axisL = self.scsz*30
        borderDist = self.scsz*20

        # Side view axis widget
        self.sideViewCanvas.create_line( self.canvasW - (borderDist + axisL), borderDist + axisL, self.canvasW - borderDist, borderDist + axisL,
                                            fill = "red", width = axisW, tag = "alwaysShown" )  # x-axis
        self.sideViewCanvas.create_text( self.canvasW - (borderDist + axisL), borderDist + axisL + self.scsz*10, text = "X",
                                            font = self.defaultFont, fill = "red", tag = "alwaysShown" )
        self.sideViewCanvas.create_line( self.canvasW - borderDist, borderDist, self.canvasW - borderDist, borderDist + axisL,
                                            fill = "blue", width = axisW, tag = "alwaysShown" )  # z-axis
        self.sideViewCanvas.create_text( self.canvasW - borderDist + self.scsz*10, borderDist, text = "Z",
                                            font = self.defaultFont, fill = "blue", tag = "alwaysShown" )

        # Front view axis widget
        self.frontViewCanvas.create_line( self.canvasW - (borderDist + axisL), borderDist + axisL, self.canvasW - borderDist, borderDist + axisL,
                                             fill = "green", width = axisW, tag = "alwaysShown" )  # y-axis
        self.frontViewCanvas.create_text( self.canvasW - borderDist, borderDist + axisL + self.scsz*10, text = "Y",
                                             font = self.defaultFont, fill = "green", tag = "alwaysShown" )
        self.frontViewCanvas.create_line( self.canvasW - (borderDist + axisL), borderDist, self.canvasW - (borderDist + axisL), borderDist + axisL,
                                             fill = "blue", width = axisW, tag = "alwaysShown" )  # z-axis
        self.frontViewCanvas.create_text( self.canvasW - (borderDist + axisL) - self.scsz*10, borderDist, text = "Z",
                                             font = self.defaultFont, fill = "blue", tag = "alwaysShown" )

        # Top view axis widget
        self.topViewCanvas.create_line( self.canvasW - (borderDist + axisL), borderDist, self.canvasW - borderDist, borderDist,
                                           fill = "red", width = axisW, tag = "alwaysShown" )  # x-axis
        self.topViewCanvas.create_text( self.canvasW - (borderDist + axisL), borderDist - self.scsz*10, text = "X",
                                           font = self.defaultFont, fill = "red", tag = "alwaysShown" )
        self.topViewCanvas.create_line( self.canvasW - borderDist, borderDist, self.canvasW - borderDist, borderDist + axisL,
                                           fill = "green", width = axisW, tag = "alwaysShown" )  # y-axis
        self.topViewCanvas.create_text( self.canvasW - borderDist + self.scsz*10, borderDist + axisL, text = "Y",
                                           font = self.defaultFont, fill = "green", tag = "alwaysShown" )

        # Origin point on canvas
        r = self.scsz*3
        fillCol = "black"
        borderCol = "black"
        w = self.scsz*1
        self.sideViewCanvas.create_oval( self.canvasW - r + self.canvasOffset[0], self.canvasH - r + self.canvasOffset[1],
                                            self.canvasW + r + self.canvasOffset[0], self.canvasH + r + self.canvasOffset[1],
                                            fill = fillCol, outline = borderCol, width = w, tag = "alwaysShown" )


    def redraw(self):
        # Redraw views
        self.sideViewCanvas.delete("clear")
        self.frontViewCanvas.delete("clear")
        self.topViewCanvas.delete("clear")

        # Spine
        for j in range(2, -1, -2):  # Skip dummy joint
            self.drawJoint( self.robot.spine.joints[j].id,
                            self.robot.spine.joints[j].tfJointInWorld[0, 3],
                            self.robot.spine.joints[j].tfJointInWorld[1, 3],
                            self.robot.spine.joints[j].tfJointInWorld[2, 3] )

        # Legs
        for leg in reversed(self.robot.legs):
            for j in range(4, -1, -1):
                self.drawLink( leg.joints[j].tfJointInWorld[0, 3],
                               leg.joints[j].tfJointInWorld[1, 3],
                               leg.joints[j].tfJointInWorld[2, 3],
                               leg.joints[j+1].tfJointInWorld[0, 3],
                               leg.joints[j+1].tfJointInWorld[1, 3],
                               leg.joints[j+1].tfJointInWorld[2, 3] )
            for j in range(4, -1, -1):
                self.drawJoint( leg.joints[j].id,
                                leg.joints[j].tfJointInWorld[0, 3],
                                leg.joints[j].tfJointInWorld[1, 3],
                                leg.joints[j].tfJointInWorld[2, 3] )
            self.drawEE( leg.joints[5].id,
                         leg.joints[5].tfJointInWorld[0, 3],
                         leg.joints[5].tfJointInWorld[1, 3],
                         leg.joints[5].tfJointInWorld[2, 3] )

        # Target
        if self.showTargets:
            for i, target in enumerate(self.robot.targets):
                self.drawTarget(target, self.robot.speeds[i])


    def drawJoint(self, id, x, y, z):
        r = self.scsz*13
        fillCol = "#FFFFE0"
        borderCol = "#00008B"
        w = self.scsz*3
        self.sideViewCanvas.create_oval( self.canvasW - self.canvasScale*x - r + self.canvasOffset[0], self.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                                            self.canvasW - self.canvasScale*x + r + self.canvasOffset[0], self.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                                            fill = fillCol, outline = borderCol, width = w, tag = "clear" )
        self.sideViewCanvas.create_text( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                            text = id, font = ("Times", 12, "bold"), tag = "clear" )

        self.frontViewCanvas.create_oval( self.canvasW + self.canvasScale*y - r + self.canvasOffset[0], self.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                                            self.canvasW + self.canvasScale*y + r + self.canvasOffset[0], self.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                                             fill = fillCol, outline = borderCol, width = w, tag = "clear" )
        self.frontViewCanvas.create_text( self.canvasW + self.canvasScale*y + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                             text = id, font = ("Times", 12, "bold"), tag = "clear" )

        self.topViewCanvas.create_oval( self.canvasW - self.canvasScale*x - r + self.canvasOffset[0], self.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
                                           self.canvasW - self.canvasScale*x + r + self.canvasOffset[0], self.canvasH + self.canvasScale*y + r + self.canvasOffset[2],
                                           fill = fillCol, outline = borderCol, width = w, tag = "clear" )
        self.topViewCanvas.create_text( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH + self.canvasScale*y + self.canvasOffset[2],
                                           text = id, font = ("Times", 12, "bold"), tag = "clear" )


    def drawEE(self, id, x, y, z):
        r = self.scsz*13
        fillCol = "#00008B"
        borderCol = "#00008B"
        w = self.scsz*3
        self.sideViewCanvas.create_oval( self.canvasW - self.canvasScale*x - r + self.canvasOffset[0], self.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                                            self.canvasW - self.canvasScale*x + r + self.canvasOffset[0], self.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                                            fill = fillCol, outline = borderCol, width = w, tag = "clear" )
        self.sideViewCanvas.create_text( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                            text = id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )

        self.frontViewCanvas.create_oval( self.canvasW + self.canvasScale*y - r + self.canvasOffset[0], self.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                                             self.canvasW + self.canvasScale*y + r + self.canvasOffset[0], self.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                                             fill = fillCol, outline = borderCol, width = w, tag = "clear" )
        self.frontViewCanvas.create_text( self.canvasW + self.canvasScale*y + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                             text = id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )

        self.topViewCanvas.create_oval( self.canvasW - self.canvasScale*x - r + self.canvasOffset[0], self.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
                                           self.canvasW - self.canvasScale*x + r + self.canvasOffset[0], self.canvasH + self.canvasScale*y + r + self.canvasOffset[2],
                                           fill = fillCol, outline = borderCol, width = w, tag = "clear" )
        self.topViewCanvas.create_text( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH + self.canvasScale*y + self.canvasOffset[2],
                                           text = id, fill = "white", font = ("Times", 12, "bold"), tag = "clear" )


    def drawLink(self, Ax, Ay, Az, Bx, By, Bz):
        fillCol = "#00008B"
        w = self.scsz*5
        self.sideViewCanvas.create_line( self.canvasW - self.canvasScale*Ax + self.canvasOffset[0], self.canvasH - self.canvasScale*Az + self.canvasOffset[1],
                                            self.canvasW - self.canvasScale*Bx + self.canvasOffset[0], self.canvasH - self.canvasScale*Bz + self.canvasOffset[1],
                                            fill = fillCol, width = w, tag = "clear" )
        self.frontViewCanvas.create_line( self.canvasW + self.canvasScale*Ay + self.canvasOffset[0], self.canvasH - self.canvasScale*Az + self.canvasOffset[1],
                                             self.canvasW + self.canvasScale*By + self.canvasOffset[0], self.canvasH - self.canvasScale*Bz + self.canvasOffset[1],
                                             fill = fillCol, width = w, tag = "clear" )
        self.topViewCanvas.create_line( self.canvasW - self.canvasScale*Ax + self.canvasOffset[0], self.canvasH + self.canvasScale*Ay + self.canvasOffset[2],
                                            self.canvasW - self.canvasScale*Bx + self.canvasOffset[0], self.canvasH + self.canvasScale*By + self.canvasOffset[2],
                                            fill = fillCol, width = w, tag = "clear" )


    def drawTarget(self, target, speed):
        # Target circle
        r = self.scsz*16
        borderCol = "green"
        w = self.scsz*5
        x = target[0, 3]
        y = target[1, 3]
        z = target[2, 3]
        self.sideViewCanvas.create_oval( self.canvasW - self.canvasScale*x - r + self.canvasOffset[0], self.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                                            self.canvasW - self.canvasScale*x + r + self.canvasOffset[0], self.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                                            outline = borderCol, width = w, tag = "clear" )
        self.frontViewCanvas.create_oval( self.canvasW + self.canvasScale*y - r + self.canvasOffset[0], self.canvasH - self.canvasScale*z - r + self.canvasOffset[1],
                                             self.canvasW + self.canvasScale*y + r + self.canvasOffset[0], self.canvasH - self.canvasScale*z + r + self.canvasOffset[1],
                                             outline = borderCol, width = w, tag = "clear" )
        self.topViewCanvas.create_oval( self.canvasW - self.canvasScale*x - r + self.canvasOffset[0], self.canvasH + self.canvasScale*y - r + self.canvasOffset[2],
                                           self.canvasW - self.canvasScale*x + r + self.canvasOffset[0], self.canvasH + self.canvasScale*y + r + self.canvasOffset[2],
                                           outline = borderCol, width = w, tag = "clear" )
        # Line along X
        tmpVec = np.array([50, 0, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        fillCol = "red"
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.sideViewCanvas.create_line( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                            self.canvasW - self.canvasScale*lx + self.canvasOffset[0], self.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                                            fill = fillCol, width = w, tag = "clear" )
        self.frontViewCanvas.create_line( self.canvasW + self.canvasScale*y + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                             self.canvasW + self.canvasScale*ly + self.canvasOffset[0], self.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                                             fill = fillCol, width = w, tag = "clear" )
        self.topViewCanvas.create_line( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH + self.canvasScale*y + self.canvasOffset[2],
                                           self.canvasW - self.canvasScale*lx + self.canvasOffset[0], self.canvasH + self.canvasScale*ly + self.canvasOffset[2],
                                           fill = fillCol, width = w, tag = "clear" )
        # Line along Y
        tmpVec = np.array([0, 50, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        fillCol = "green"
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.sideViewCanvas.create_line( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                            self.canvasW - self.canvasScale*lx + self.canvasOffset[0], self.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                                            fill = fillCol, width = w, tag = "clear" )
        self.frontViewCanvas.create_line( self.canvasW + self.canvasScale*y + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                             self.canvasW + self.canvasScale*ly + self.canvasOffset[0], self.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                                             fill = fillCol, width = w, tag = "clear" )
        self.topViewCanvas.create_line( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH + self.canvasScale*y + self.canvasOffset[2],
                                           self.canvasW - self.canvasScale*lx + self.canvasOffset[0], self.canvasH + self.canvasScale*ly + self.canvasOffset[2],
                                           fill = fillCol, width = w, tag = "clear" )
        # Line along Z
        tmpVec = np.array([0, 0, 50, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        fillCol = "blue"
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.sideViewCanvas.create_line( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                            self.canvasW - self.canvasScale*lx + self.canvasOffset[0], self.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                                            fill = fillCol, width = w, tag = "clear" )
        self.frontViewCanvas.create_line( self.canvasW + self.canvasScale*y + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                             self.canvasW + self.canvasScale*ly + self.canvasOffset[0], self.canvasH - self.canvasScale*lz + self.canvasOffset[1],
                                             fill = fillCol, width = w, tag = "clear" )
        self.topViewCanvas.create_line( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH + self.canvasScale*y + self.canvasOffset[2],
                                           self.canvasW - self.canvasScale*lx + self.canvasOffset[0], self.canvasH + self.canvasScale*ly + self.canvasOffset[2],
                                           fill = fillCol, width = w, tag = "clear" )
        # Speed vector
        fillCol = "#39FF14"
        sx = speed[0]
        sy = speed[1]
        sz = speed[2]
        k = 500.0 / self.inputHandler.inputForceMax  # Arbitrary scaling, to make max. length of vector constant
        self.sideViewCanvas.create_line( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                            self.canvasW - self.canvasScale*x - sx*k + self.canvasOffset[0], self.canvasH - self.canvasScale*z - sz*k + self.canvasOffset[1],
                                            fill = fillCol, width = w, tag = "clear" )
        self.frontViewCanvas.create_line( self.canvasW + self.canvasScale*y + self.canvasOffset[0], self.canvasH - self.canvasScale*z + self.canvasOffset[1],
                                             self.canvasW + self.canvasScale*y + sy*k + self.canvasOffset[0], self.canvasH - self.canvasScale*z - sz*k + self.canvasOffset[1],
                                             fill = fillCol, width = w, tag = "clear" )
        self.topViewCanvas.create_line( self.canvasW - self.canvasScale*x + self.canvasOffset[0], self.canvasH + self.canvasScale*y + self.canvasOffset[2],
                                           self.canvasW - self.canvasScale*x - sx*k + self.canvasOffset[0], self.canvasH + self.canvasScale*y + sy*k + self.canvasOffset[2],
                                           fill = fillCol, width = w, tag = "clear" )
