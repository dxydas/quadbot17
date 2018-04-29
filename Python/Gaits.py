import csv
import numpy as np
from time import time, sleep


class Gaits():
    def __init__(self, robot):
        self.robot = robot

        # Input vars
        self.FLUpDown = []
        self.FLFwdBack = []
        self.FRUpDown = []
        self.FRFwdBack = []
        self.RLUpDown = []
        self.RLFwdBack = []
        self.RRUpDown = []
        self.RRFwdBack = []
        self.currentPose = []


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
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for r, row in enumerate(reader):
                if r in range(rowOffset, rowOffset + arraySize):
                    #print(r, row)
                    for c, col in enumerate(row):
                        if c in range(2, 10):
                            #print(c, col)
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
        for t in range(0, len(self.FLUpDown)):

            distances = np.zeros(len(self.FLUpDown))
            minThresh = 20
            penalty = 10

            x = abs(self.currentPose[0] - self.FLUpDown[t])
            distances[0] = x
            if (x > minThresh):
                distances[0] += penalty

            x = abs(self.currentPose[1] - self.FLFwdBack[t])
            distances[1] = x
            if (x > minThresh):
                distances[1] += penalty

            x = abs(self.currentPose[2] - self.FRUpDown[t])
            distances[2] = x
            if (x > minThresh):
                distances[2] += penalty

            x = abs(self.currentPose[3] - self.FRFwdBack[t])
            distances[3] = x
            if (x > minThresh):
                distances[3] += penalty

            x = abs(self.currentPose[4] - self.RLUpDown[t])
            distances[4] = x
            if (x > minThresh):
                distances[4] += penalty

            x = abs(self.currentPose[5] - self.RLFwdBack[t])
            distances[5] = x
            if (x > minThresh):
                distances[5] += penalty

            x = abs(self.currentPose[6] - self.RRUpDown[t])
            distances[6] = x
            if (x > minThresh):
                distances[6] += penalty

            x = abs(self.currentPose[7] - self.RRFwdBack[t])
            distances[7] = x
            if (x > minThresh):
                distances[7] += penalty

            distanceMetric = 0
            for d in distances:
                distanceMetric += d

            if (t == 0) or (distanceMetric < minDist):
                minDist = distanceMetric
                idx = t

        #print("Closest new index:", idx)
        #print("Dist:", minDist)

        return idx


    def loadTargetsStep(self, t):
        xAdjust = -20
        zAdjust = 20

        # FL
        i = 0
        self.robot.legTargets[i][0, 3] = self.robot.legTargetsHome[i][0, 3] + self.FLFwdBack[t] + xAdjust
        self.robot.legTargets[i][1, 3] = self.robot.legTargetsHome[i][1, 3]
        self.robot.legTargets[i][2, 3] = self.robot.legTargetsHome[i][2, 3] + self.FLUpDown[t] + zAdjust
        self.robot.runLegIK(i)
        # FR
        i = 1
        self.robot.legTargets[i][0, 3] = self.robot.legTargetsHome[i][0, 3] + self.FRFwdBack[t] + xAdjust
        self.robot.legTargets[i][1, 3] = self.robot.legTargetsHome[i][1, 3]
        self.robot.legTargets[i][2, 3] = self.robot.legTargetsHome[i][2, 3] + self.FRUpDown[t] + zAdjust
        self.robot.runLegIK(i)
        # RL
        i = 2
        self.robot.legTargets[i][0, 3] = self.robot.legTargetsHome[i][0, 3] + self.RLFwdBack[t] + xAdjust
        self.robot.legTargets[i][1, 3] = self.robot.legTargetsHome[i][1, 3]
        self.robot.legTargets[i][2, 3] = self.robot.legTargetsHome[i][2, 3] + self.RLUpDown[t] + zAdjust
        self.robot.runLegIK(i)
        # RR
        i = 3
        self.robot.legTargets[i][0, 3] = self.robot.legTargetsHome[i][0, 3] + self.RRFwdBack[t] + xAdjust
        self.robot.legTargets[i][1, 3] = self.robot.legTargetsHome[i][1, 3]
        self.robot.legTargets[i][2, 3] = self.robot.legTargetsHome[i][2, 3] + self.RRUpDown[t] + zAdjust
        self.robot.runLegIK(i)

        self.savePose(t)

        #print("Current index:", t)
