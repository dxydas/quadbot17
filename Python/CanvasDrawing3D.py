from Globals import showTargets

from matplotlib import use
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
import sys
if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk


class CanvasDrawing3D():
    def __init__(self, defaultFont, canvas, robot, inputHandler):
        self.defaultFont = defaultFont
        self.canvas = canvas
        self.robot = robot
        self.inputHandler = inputHandler

        self.figure = None
        self.figureCanvas = None
        self.axes = None
        #self.toolbar = None

        self.initViews()


    def run(self):
        self.ani = animation.FuncAnimation(self.figure, self.redraw, interval=100)


    def initViews(self):
        # Interactive backend for TkInter: Agg rendering to a Tk canvas
        use("TkAgg")

        # Figure
        self.figure = Figure(figsize=(8,4), dpi=200, tight_layout=True)
        self.figureCanvas = FigureCanvasTkAgg(self.figure, self.canvas)

        # Axes with 3D projection
        self.axes = self.figure.add_subplot(111, projection='3d')

        # Draw/resize
        self.figureCanvas.draw()
        self.figureCanvas.get_tk_widget().pack(side=Tk.BOTTOM, fill=Tk.BOTH, expand=True)

        # Toolbar
        # Deprecated in matplotlib v2.2
        #self.toolbar = NavigationToolbar2TkAgg(self.figureCanvas, self.canvas)
        #self.toolbar.update()
        #self.figureCanvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=True)


    def redraw(self, frame):
        # Clear axes
        self.axes.clear()

        # Spine
        n = len(self.robot.spine.joints) - 1
        ids = [""]*n
        xs = np.zeros(n)
        ys = np.zeros(n)
        zs = np.zeros(n)
        for i, j in enumerate(range(0, n+1, 2)):  # Skip dummy joint
            ids[i] = self.robot.spine.joints[j].id
            xs[i] = self.robot.spine.joints[j].tfJointInWorld[0, 3]
            ys[i] = self.robot.spine.joints[j].tfJointInWorld[1, 3]
            zs[i] = self.robot.spine.joints[j].tfJointInWorld[2, 3]
        self.drawJoints(ids, xs, ys, zs)
        self.drawLinks(xs, ys, zs)

        # Legs
        for leg in self.robot.legs:
            n = len(leg.joints)
            ids = [""]*n
            xs = np.zeros(n)
            ys = np.zeros(n)
            zs = np.zeros(n)
            for j in range(0, n):
                ids[j] = str(leg.joints[j].id)
                xs[j] = leg.joints[j].tfJointInWorld[0, 3]
                ys[j] = leg.joints[j].tfJointInWorld[1, 3]
                zs[j] = leg.joints[j].tfJointInWorld[2, 3]
            self.drawJoints(ids, xs, ys, zs)
            self.drawLinks(xs, ys, zs)

        # Target
        if showTargets:
            for i, target in enumerate(self.robot.targets):
                self.drawTarget(target, self.robot.speeds[i])


    def drawJoints(self, ids, xs, ys, zs):
        #for j, id in enumerate(ids):
            #self.axes.text(xs[j], ys[j], zs[j], id)
        self.axes.scatter(xs, ys, zs, marker='o', s=200)


    def drawLinks(self, xs, ys, zs):
        self.axes.plot(xs, ys, zs, linewidth=4)


    def drawTarget(self, target, speed):
        # Target circle
        x = target[0, 3]
        y = target[1, 3]
        z = target[2, 3]
        # TODO: Draw

        # Line along X
        tmpVec = np.array([50, 0, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        fillCol = "red"
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        # TODO: Draw

        # Line along Y
        tmpVec = np.array([0, 50, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        # TODO: Draw

        # Line along Z
        tmpVec = np.array([0, 0, 50, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        # TODO: Draw

        # Speed vector
        sx = speed[0]
        sy = speed[1]
        sz = speed[2]
        k = 500.0 / self.inputHandler.inputForceMax  # Arbitrary scaling, to make max. length of vector constant
        # TODO: Draw
