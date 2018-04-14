import Params

from matplotlib import use
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import juggle_axes
import matplotlib.animation as animation
import numpy as np
import sys
if sys.version_info[0] < 3:
    import Tkinter as Tk
else:
    import tkinter as Tk


class CanvasDrawing3D():
    def __init__(self, scsz, defaultFont, canvas, robot):
        self.scsz = scsz
        self.defaultFont = defaultFont
        self.canvas = canvas
        self.robot = robot

        self.figure = None
        self.figureCanvas = None
        self.axes = None
        #self.toolbar = None

        # Elements
        self.allSpineElements = [JointElements() for x in range(2)]
        self.allJointElements = [JointElements() for x in range(4)]
        self.allEndEffectorElements = [JointElements() for x in range(4)]
        self.allLinkElements = [LinkElements() for x in range(4)]
        self.allTargetElements = [TargetElements() for x in range(4)]

        self.initViews()

        self.ani = animation.FuncAnimation(self.figure, self.redraw, interval=40)


    def initViews(self):
        # Interactive backend for TkInter: Agg rendering to a Tk canvas
        use("TkAgg")

        # Figure
        self.figure = Figure(figsize=(self.scsz*4, self.scsz*2), dpi=self.scsz*100, tight_layout=True)
        self.figureCanvas = FigureCanvasTkAgg(self.figure, self.canvas)

        # Axes with 3D projection
        self.axes = self.figure.add_subplot(111, projection='3d')

        # Set up view
        self.axes.view_init(azim=60, elev=45)
        xs = -250
        ys = -200
        zs = -300
        d = 400
        self.axes.set_xlim3d(xs, xs+d)
        self.axes.set_ylim3d(ys, ys+d)
        self.axes.set_zlim3d(zs, zs+d)

        # Draw/resize
        self.figureCanvas.draw()
        self.figureCanvas.get_tk_widget().pack(side=Tk.BOTTOM, fill=Tk.BOTH, expand=True)

        # Toolbar
        # Deprecated in matplotlib v2.2
        #self.toolbar = NavigationToolbar2TkAgg(self.figureCanvas, self.canvas)
        #self.toolbar.update()
        #self.figureCanvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=True)

        # Origin point on canvas
        origin = self.axes.scatter(0.0, 0.0, 0.0, marker='o', s=50, c='black', alpha=0.5)

        # Draw elements for the first time, store handles
        spineIdx = 0
        jointIdx = 0
        endEffectorIdx = 0
        linkIdx = 0
        targetIdx = 0
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
        self.drawSpine(spineIdx, ids, xs, ys, zs)
        spineIdx = spineIdx + 1

        # Legs
        for leg in self.robot.legs:
            n = len(leg.joints) - 1
            ids = [""]*n
            xs = np.zeros(n)
            ys = np.zeros(n)
            zs = np.zeros(n)
            for j in range(0, n):
                ids[j] = str(leg.joints[j].id)
                xs[j] = leg.joints[j].tfJointInWorld[0, 3]
                ys[j] = leg.joints[j].tfJointInWorld[1, 3]
                zs[j] = leg.joints[j].tfJointInWorld[2, 3]
            self.drawJoints(jointIdx, ids, xs, ys, zs)
            jointIdx = jointIdx + 1
            self.drawLinks(linkIdx, xs, ys, zs)
            linkIdx = linkIdx + 1
            j = n
            ids = [""]*1
            xs = np.zeros(1)
            ys = np.zeros(1)
            zs = np.zeros(1)
            ids[0] = str(leg.joints[j].id)
            xs[0] = leg.joints[j].tfJointInWorld[0, 3]
            ys[0] = leg.joints[j].tfJointInWorld[1, 3]
            zs[0] = leg.joints[j].tfJointInWorld[2, 3]
            self.drawEndEffectors(endEffectorIdx, ids, xs, ys, zs)
            endEffectorIdx = endEffectorIdx + 1

        # Targets
        for i, target in enumerate(self.robot.targets):
            self.drawTarget(targetIdx, target, self.robot.speeds[i])
            targetIdx = targetIdx + 1


    def redraw(self, frame):
        spineIdx = 0
        jointIdx = 0
        endEffectorIdx = 0
        linkIdx = 0
        targetIdx = 0
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
        self.moveSpine(spineIdx, ids, xs, ys, zs)
        spineIdx = spineIdx + 1

        # Legs
        for leg in self.robot.legs:
            n = len(leg.joints) - 1
            ids = [""]*n
            xs = np.zeros(n)
            ys = np.zeros(n)
            zs = np.zeros(n)
            for j in range(0, n):
                ids[j] = str(leg.joints[j].id)
                xs[j] = leg.joints[j].tfJointInWorld[0, 3]
                ys[j] = leg.joints[j].tfJointInWorld[1, 3]
                zs[j] = leg.joints[j].tfJointInWorld[2, 3]
            self.moveJoints(jointIdx, ids, xs, ys, zs)
            jointIdx = jointIdx + 1
            self.moveLinks(linkIdx, xs, ys, zs)
            linkIdx = linkIdx + 1
            j = n
            ids = [""]*1
            xs = np.zeros(1)
            ys = np.zeros(1)
            zs = np.zeros(1)
            ids[0] = str(leg.joints[j].id)
            xs[0] = leg.joints[j].tfJointInWorld[0, 3]
            ys[0] = leg.joints[j].tfJointInWorld[1, 3]
            zs[0] = leg.joints[j].tfJointInWorld[2, 3]
            self.moveEndEffectors(endEffectorIdx, ids, xs, ys, zs)
            endEffectorIdx = endEffectorIdx + 1

        # Targets
        for i, target in enumerate(self.robot.targets):
            self.toggleTarget(targetIdx, Params.showTargets)
            if Params.showTargets:
                self.moveTarget(targetIdx, target, self.robot.speeds[i])
            targetIdx = targetIdx + 1


    def drawSpine(self, index, ids, xs, ys, zs):
        self.allSpineElements[index].circles = self.axes.scatter(xs, ys, zs, marker='o', s=200, c='cyan', alpha=0.7)
        for j, id in enumerate(ids):
            self.allSpineElements[index].texts.append(
                self.axes.text(xs[j], ys[j], zs[j], id, zdir=None) )


    def drawJoints(self, index, ids, xs, ys, zs):
        self.allJointElements[index].circles = self.axes.scatter(xs, ys, zs, marker='o', s=200, alpha=0.7)
        for j, id in enumerate(ids):
            self.allJointElements[index].texts.append(
                self.axes.text(xs[j], ys[j], zs[j], id, zdir=None) )


    def drawEndEffectors(self, index, ids, xs, ys, zs):
        self.allEndEffectorElements[index].circles = self.axes.scatter(xs, ys, zs, marker='o', s=200, c='blue', alpha=0.5)
        for j, id in enumerate(ids):
            self.allEndEffectorElements[index].texts.append(
                self.axes.text(xs[j], ys[j], zs[j], id, zdir=None) )


    def drawLinks(self, index, xs, ys, zs):
        self.allLinkElements[index].lines = self.axes.plot(xs, ys, zs, linewidth=4)[0]


    def drawTarget(self, index, target, speed):
        # Target circle
        x = target[0, 3]
        y = target[1, 3]
        z = target[2, 3]
        self.allTargetElements[index].circles = self.axes.scatter(x, y, z, marker='o', s=600, c="green", alpha=0.5)

        # Line along X
        tmpVec = np.array([50, 0, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.allTargetElements[index].frameXLines = self.axes.plot([x, lx], [y, ly], [z, lz], linewidth=2, c="red")[0]

        # Line along Y
        tmpVec = np.array([0, 50, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.allTargetElements[index].frameYLines = self.axes.plot([x, lx], [y, ly], [z, lz], linewidth=2, c="green")[0]

        # Line along Z
        tmpVec = np.array([0, 0, 50, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.allTargetElements[index].frameZLines = self.axes.plot([x, lx], [y, ly], [z, lz], linewidth=2, c="blue")[0]

        # Speed vector
        sx = speed[0]
        sy = speed[1]
        sz = speed[2]
        # Arbitrary scaling, to make max. length of vector constant
        k = 500.0 / Params.inputForceMax
        self.allTargetElements[index].speedLines = self.axes.plot([x, x+sx], [y, y+sy], [z, z+sz], linewidth=2, c="#39FF14")[0]


    def moveSpine(self, index, ids, xs, ys, zs):
        self.allSpineElements[index].circles._offsets3d = juggle_axes(xs, ys, zs, 'z')
        for j, id in enumerate(ids):
            self.allSpineElements[index].texts[j].set_position([xs[j], ys[j]])
            self.allSpineElements[index].texts[j].set_3d_properties(zs[j], None)


    def moveJoints(self, index, ids, xs, ys, zs):
        self.allJointElements[index].circles._offsets3d = juggle_axes(xs, ys, zs, 'z')
        for j, id in enumerate(ids):
            self.allJointElements[index].texts[j].set_position([xs[j], ys[j]])
            self.allJointElements[index].texts[j].set_3d_properties(zs[j], None)


    def moveEndEffectors(self, index, ids, xs, ys, zs):
        self.allEndEffectorElements[index].circles._offsets3d = juggle_axes(xs, ys, zs, 'z')
        for j, id in enumerate(ids):
            self.allEndEffectorElements[index].texts[j].set_position([xs[j], ys[j]])
            self.allEndEffectorElements[index].texts[j].set_3d_properties(zs[j], None)


    def moveLinks(self, index, xs, ys, zs):
        self.allLinkElements[index].lines.set_data(xs, ys)
        self.allLinkElements[index].lines.set_3d_properties(zs, 'z')


    def moveTarget(self, index, target, speed):
        # Target circle
        x = target[0, 3]
        y = target[1, 3]
        z = target[2, 3]
        self.allTargetElements[index].circles._offsets3d = juggle_axes([x], [y], [z], 'z')

        # Line along X
        tmpVec = np.array([50, 0, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.allTargetElements[index].frameXLines.set_data([x, lx], [y, ly])
        self.allTargetElements[index].frameXLines.set_3d_properties([z, lz], 'z')

        # Line along Y
        tmpVec = np.array([0, 50, 0, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.allTargetElements[index].frameYLines.set_data([x, lx], [y, ly])
        self.allTargetElements[index].frameYLines.set_3d_properties([z, lz], 'z')

        # Line along Z
        tmpVec = np.array([0, 0, 50, 1]).reshape(4, 1)
        tmpVec = target * tmpVec
        lx = tmpVec[0, 0]
        ly = tmpVec[1, 0]
        lz = tmpVec[2, 0]
        self.allTargetElements[index].frameZLines.set_data([x, lx], [y, ly])
        self.allTargetElements[index].frameZLines.set_3d_properties([z, lz], 'z')

        # Speed vector
        sx = speed[0]
        sy = speed[1]
        sz = speed[2]
        # Arbitrary scaling, to make max. length of vector constant
        k = 500.0 / Params.inputForceMax
        self.allTargetElements[index].speedLines.set_data([x, x+sx], [y, y+sy])
        self.allTargetElements[index].speedLines.set_3d_properties([z, z+sz], 'z')


    def toggleTarget(self, index, show):
        self.allTargetElements[index].circles.set_visible(show)
        self.allTargetElements[index].frameXLines.set_visible(show)
        self.allTargetElements[index].frameYLines.set_visible(show)
        self.allTargetElements[index].frameZLines.set_visible(show)
        self.allTargetElements[index].speedLines.set_visible(show)


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
