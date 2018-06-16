from HelperFunctions import applyYawPitchRoll

import csv
import numpy as np
from time import time, sleep


class Gaits():
    def __init__(self, robot):
        self.robot = robot

        # Input vars
        self.numOfRows = 100
        self.numOfCols = 28
        self.isDataLoaded = False
        self.gaitData = np.zeros( (self.numOfRows, self.numOfCols) )


    def savePose(self, i):
        self.currentPose = self.gaitData[i, :]


    def loadFromFile(self, filename):
        rowOffset = 2
        colOffset = 1
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for r, row in enumerate(reader):
                if r in range(rowOffset, rowOffset + self.numOfRows):
                    self.gaitData[r-rowOffset, :] = row[colOffset : colOffset + self.numOfCols]
        csvfile.close()
        self.isDataLoaded = True

        amplAdjust = [40, 40, 50]  # X, Y, Z

        # Leg adjustments
        m = 5  # Leg X, Y, Z, Roll, Pitch
        n = 4  # Num. of legs
        for i in range(0, n):
            for j in range(0, 3):
                self.gaitData[:, j + i*m] *= amplAdjust[j]

        # Base adjustments
        for j in range(0, 3):
            self.gaitData[:, j + m*n] *= amplAdjust[j]


    def findClosestLegPose(self):
        minThresh = 20
        penalty = 10
        minDist = 0
        idx = 0
        for r in range(0, self.numOfRows):
            distances = np.zeros(self.numOfCols)
            for c in range(0, self.numOfCols):
                x = abs(self.currentPose[c] - self.gaitData[r, c])
                distances[c] = x
                if (x > minThresh):
                    distances[c] += penalty

            distanceMetric = 0
            for d in distances:
                distanceMetric += d

            if (r == 0) or (distanceMetric < minDist):
                minDist = distanceMetric
                idx = r

        #print("Closest new index:", idx)
        #print("Dist:", minDist)

        return idx


    def loadTargetsStep(self, t, spineDeflection=0, adjustRP=False):

        posAdjust = [-20, 0, 20]

        m = 5  # Leg X, Y, Z, Roll, Pitch
        n = 4  # Num. of legs
        for i in range(0, n):
            for j in range(0, 3):
                self.robot.legTargets[i][j, 3] = self.robot.legTargetsHome[i][j, 3] + self.gaitData[t, j + i*m] + posAdjust[j]
            roll = self.gaitData[t, 3 + i*m]
            pitch = self.gaitData[t, 4 + i*m]
            applyYawPitchRoll(self.robot.legTargets[i], 0.0, pitch, roll)
            #self.robot.runLegIK(i)

        for j in range(0, 3):
            self.robot.baseTarget[j, 3] = self.robot.baseTargetHome[j, 3] + self.gaitData[t, j + m*n]

        roll = self.gaitData[t, 23]
        pitch = self.gaitData[t, 24]
        yaw = self.gaitData[t, 25]
        applyYawPitchRoll(self.robot.baseTarget, yaw, pitch, roll)

        self.robot.spine.angles[0] = self.gaitData[t, 26] + spineDeflection
        self.robot.spine.angles[2] = self.gaitData[t, 27]

        if adjustRP:
            # Roll/pitch adjustment
            roll = (self.robot.spineAngleOffsets[0] + self.robot.spine.angles[0]) / 2.0
            pitch = (self.robot.spineAngleOffsets[2] - self.robot.spine.angles[2]) / 2.0
            applyYawPitchRoll(self.robot.baseTarget, yaw, pitch, roll)

        self.robot.moveBase()

        self.savePose(t)

        #print("Current index:", t)
