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
        self.gaitData = np.zeros( (self.numOfRows, self.numOfCols) )


    def savePose(self, i):
        self.currentPose = self.gaitData[i, :]


    def loadFromFile(self, filename):
        rowOffset = 2
        colOffset = 2
        upDownAmplAdjust = 50
        fwdBackAmplAdjust = 40
        inOutAmplAdjust = 40
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for r, row in enumerate(reader):
                if r in range(rowOffset, rowOffset + self.numOfRows):
                    self.gaitData[r-rowOffset, :] = row[colOffset : colOffset + self.numOfCols]
        csvfile.close()

        # Leg adjustments
        m = 5  # Leg Z, X, Y, Roll, Pitch
        n = 4  # Num. of legs
        for i in range(0, n):
            self.gaitData[:, 0 + i*m] *= upDownAmplAdjust
            self.gaitData[:, 1 + i*m] *= fwdBackAmplAdjust
            self.gaitData[:, 2 + i*m] *= inOutAmplAdjust

        # Base adjustments
        self.gaitData[:, 20] *= upDownAmplAdjust
        self.gaitData[:, 21] *= fwdBackAmplAdjust
        self.gaitData[:, 22] *= inOutAmplAdjust


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


    def loadTargetsStep(self, t):
        xAdjust = -20
        yAdjust = 0
        zAdjust = 20

        m = 5  # Leg Z, X, Y, Roll, Pitch
        n = 4  # Num. of legs
        for i in range(0, n):
            self.robot.legTargets[i][0, 3] = self.robot.legTargetsHome[i][0, 3] + self.gaitData[t, 1 + i*m] + xAdjust
            self.robot.legTargets[i][1, 3] = self.robot.legTargetsHome[i][1, 3] + self.gaitData[t, 2 + i*m] + yAdjust
            self.robot.legTargets[i][2, 3] = self.robot.legTargetsHome[i][2, 3] + self.gaitData[t, 0 + i*m] + zAdjust
            roll = self.gaitData[t, 3 + i*m]
            pitch = self.gaitData[t, 4 + i*m]
            applyYawPitchRoll(self.robot.legTargets[i], 0.0, pitch, roll)
            #self.robot.runLegIK(i)

        self.robot.baseTarget[0, 3] = self.robot.baseTargetHome[0, 3] + self.gaitData[t, 21]
        self.robot.baseTarget[1, 3] = self.robot.baseTargetHome[1, 3] + self.gaitData[t, 22]
        self.robot.baseTarget[2, 3] = self.robot.baseTargetHome[2, 3] + self.gaitData[t, 20]

        roll = self.gaitData[t, 23]
        pitch = self.gaitData[t, 24]
        yaw = self.gaitData[t, 25]
        applyYawPitchRoll(self.robot.baseTarget, yaw, pitch, roll)

        self.robot.spine.angles[0] = self.gaitData[t, 26]
        self.robot.spine.angles[2] = self.gaitData[t, 27]

        self.robot.moveBase()

        self.savePose(t)

        #print("Current index:", t)
