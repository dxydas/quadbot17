import Globals
import Kinematics

import threading
import csv
import numpy as np


class Gaits():
    def __init__(self, master):
        self.master = master
        self.FLUpDown = []
        self.FLFwdBack = []
        self.FRUpDown = []
        self.FRFwdBack = []
        self.RLUpDown = []
        self.RLFwdBack = []
        self.RRUpDown = []
        self.RRFwdBack = []
        self.currentPose = []
        self.iLT = 0
        self.rateMsLT = 30
        self.gaitCallbackRunning = False


    def savePose(self, i):
        self.currentPose = [ self.FLUpDown[i], self.FLFwdBack[i], self.FRUpDown[i], self.FRFwdBack[i],
                             self.RLUpDown[i], self.RLFwdBack[i], self.RRUpDown[i], self.RRFwdBack[i] ]


    def loadFromFile(self, filename):
        self.FLUpDown = []
        self.FLFwdBack = []
        self.FRUpDown = []
        self.FRFwdBack = []
        self.RLUpDown = []
        self.RLFwdBack = []
        self.RRUpDown = []
        self.RRFwdBack = []

        arraySize = 100
        rowOffset = 2
        upDownAmplAdjust = 50
        fwdBackamplAdjust = 40
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for r, row in enumerate(reader):
                if r in range(rowOffset, rowOffset + arraySize):
                    #print r, row
                    for c, col in enumerate(row):
                        if c in range(2, 10):
                            #print c, col
                            if c == 2:
                                self.FLUpDown.append(upDownAmplAdjust*float(col))
                            if c == 3:
                                self.FLFwdBack.append(fwdBackamplAdjust*float(col))
                            if c == 4:
                                self.FRUpDown.append(upDownAmplAdjust*float(col))
                            if c == 5:
                                self.FRFwdBack.append(fwdBackamplAdjust*float(col))
                            if c == 6:
                                self.RLUpDown.append(upDownAmplAdjust*float(col))
                            if c == 7:
                                self.RLFwdBack.append(fwdBackamplAdjust*float(col))
                            if c == 8:
                                self.RRUpDown.append(upDownAmplAdjust*float(col))
                            if c == 9:
                                self.RRFwdBack.append(fwdBackamplAdjust*float(col))
        csvfile.close()


    def findClosestLegPose(self):
        minDist = 0
        idx = 0
        for i in range(0, len(self.FLUpDown)):

            distances = np.zeros(len(self.FLUpDown))
            minThresh = 20
            penalty = 10

            x = abs(self.currentPose[0] - self.FLUpDown[i])
            distances[0] = x
            if (x > minThresh):
                distances[0] += penalty

            x = abs(self.currentPose[1] - self.FLFwdBack[i])
            distances[1] = x
            if (x > minThresh):
                distances[1] += penalty

            x = abs(self.currentPose[2] - self.FRUpDown[i])
            distances[2] = x
            if (x > minThresh):
                distances[2] += penalty

            x = abs(self.currentPose[3] - self.FRFwdBack[i])
            distances[3] = x
            if (x > minThresh):
                distances[3] += penalty

            x = abs(self.currentPose[4] - self.RLUpDown[i])
            distances[4] = x
            if (x > minThresh):
                distances[4] += penalty

            x = abs(self.currentPose[5] - self.RLFwdBack[i])
            distances[5] = x
            if (x > minThresh):
                distances[5] += penalty

            x = abs(self.currentPose[6] - self.RRUpDown[i])
            distances[6] = x
            if (x > minThresh):
                distances[6] += penalty

            x = abs(self.currentPose[7] - self.RRFwdBack[i])
            distances[7] = x
            if (x > minThresh):
                distances[7] += penalty

            distanceMetric = 0
            for d in distances:
                distanceMetric += d

            if (i == 0) or (distanceMetric < minDist):
                minDist = distanceMetric
                idx = i

        #print "Current index:", self.iLT
        #print "Closest new index:", idx
        #print "Dist:", minDist
        #print "Index diff (abs):", abs(self.iLT - idx)

        return idx


    def loadTargets1(self):
        # Load from csv
        self.loadFromFile("Gait_Creep.csv")

        # Run IK
        if not 'self.gaitCallbackRunning' in globals():
            self.gaitCallbackRunning = False
        if not self.gaitCallbackRunning:
            self.iLT = 0
            self.master.after(self.rateMsLT, self.loadTargetsCallback)
        else:
            self.iLT = self.findClosestLegPose()


    def loadTargets2(self):
        # Load from csv
        self.loadFromFile("Gait_Walk.csv")

        # Run IK
        if not 'self.gaitCallbackRunning' in globals():
            self.gaitCallbackRunning = False
        if not self.gaitCallbackRunning:
            self.iLT = 0
            self.master.after(self.rateMsLT, self.loadTargetsCallback)
        else:
            self.iLT = self.findClosestLegPose()


    def loadTargetsCallback(self):
        Globals.showTargets = False
        xAdjust = -20
        zAdjust = 20
        #print "i: ", self.iLT

        if self.iLT < len(self.FLUpDown):
            # FL
            i = 0
            Globals.targets[i][0, 3] = Globals.targetsHome[i][0, 3] + self.FLFwdBack[self.iLT] + xAdjust
            Globals.targets[i][1, 3] = Globals.targetsHome[i][1, 3]
            Globals.targets[i][2, 3] = Globals.targetsHome[i][2, 3] + self.FLUpDown[self.iLT] + zAdjust
            Kinematics.runLegIK(Globals.spine, Globals.legs[i], Globals.targets[i])
            # FR
            i = 1
            Globals.targets[i][0, 3] = Globals.targetsHome[i][0, 3] + self.FRFwdBack[self.iLT] + xAdjust
            Globals.targets[i][1, 3] = Globals.targetsHome[i][1, 3]
            Globals.targets[i][2, 3] = Globals.targetsHome[i][2, 3] + self.FRUpDown[self.iLT] + zAdjust
            Kinematics.runLegIK(Globals.spine, Globals.legs[i], Globals.targets[i])
            # RL
            i = 2
            Globals.targets[i][0, 3] = Globals.targetsHome[i][0, 3] + self.RLFwdBack[self.iLT] + xAdjust
            Globals.targets[i][1, 3] = Globals.targetsHome[i][1, 3]
            Globals.targets[i][2, 3] = Globals.targetsHome[i][2, 3] + self.RLUpDown[self.iLT] + zAdjust
            Kinematics.runLegIK(Globals.spine, Globals.legs[i], Globals.targets[i])
            # RR
            i = 3
            Globals.targets[i][0, 3] = Globals.targetsHome[i][0, 3] + self.RRFwdBack[self.iLT] + xAdjust
            Globals.targets[i][1, 3] = Globals.targetsHome[i][1, 3]
            Globals.targets[i][2, 3] = Globals.targetsHome[i][2, 3] + self.RRUpDown[self.iLT] + zAdjust
            Kinematics.runLegIK(Globals.spine, Globals.legs[i], Globals.targets[i])

            self.savePose(self.iLT)

            self.iLT = self.iLT + 1
            self.gaitCallbackRunning = True
            self.master.after(self.rateMsLT, self.loadTargetsCallback)

        else:
            #print "Done"
            self.gaitCallbackRunning = False
            Globals.showTargets = True
