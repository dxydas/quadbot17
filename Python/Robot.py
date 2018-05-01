from HelperFunctions import identityTF, getRollPitchYaw, applyYawPitchRoll

import math
import numpy as np
from copy import deepcopy


class Robot():
    def __init__(self):
        self.spine = initSpine()
        self.legs = initLegs()

        # Link lengths "a-1" (last value is the foot offset)
        self.a = [0, 29.05, 76.919, 72.96, 45.032, 33.596]
        # Offsets for natural "home" position
        self.spineAngleOffsets = [0, 0, -45]
        self.legAngleOffsets = [0, -34, 67.5, -33.5, 0]

        self.baseTarget = None
        self.baseTargetHome = None
        self.baseTargetSpeed = None

        n = 4
        self.legTargets = [None]*n
        self.legTargetsHome = [None]*n
        self.legTargetSpeeds = [None]*n
        self.selectedLeg = 0

        # Dummy targets (because of moveBase()->runSpineFK->runLegIK(),
        # which finally calls runLegFK() at the end)
        for i in range(0, len(self.legs)):
            self.legTargets[i] = identityTF()

        # Base target in world
        self.baseTargetHome = identityTF()#deepcopy(self.spine.tfSpineBaseInWorld)
        self.baseTarget = deepcopy(self.baseTargetHome)
        self.baseTargetSpeed = [0, 0, 0]

        self.spine.angles = deepcopy(self.spineAngleOffsets)

        self.moveBase()

        for i, leg in enumerate(self.legs):
            leg.angles = deepcopy(self.legAngleOffsets)
            self.runLegFK(i)

        # Leg targets: Foot in world
        for i, leg in enumerate(self.legs):
            self.legTargetsHome[i] = deepcopy(leg.joints[5].tfJointInWorld)
            applyYawPitchRoll(self.legTargetsHome[i], 0, 0, 0)
            self.legTargetSpeeds[i] = [0, 0, 0]
        self.legTargets = deepcopy(self.legTargetsHome)


    def runSpineFK(self):
        d_1b = 16.975  # Dummy link offset

        s = [0, 0, 0]  # Second value is unused
        c = [0, 0, 0]
        for i in range(0, 3, 2):
            s[i] = math.sin( math.radians(self.spine.angles[i]) )
            c[i] = math.cos( math.radians(self.spine.angles[i]) )

        tfJointInPrevJoint = [0, 0, 0]

        # Front spine joint
        tfJointInPrevJoint[0] = np.matrix( [ [  c[0], -s[0],     0,     0],
                                             [  s[0],  c[0],     0,     0],
                                             [     0,     0,     1,     0],
                                             [     0,     0,     0,     1] ] )

        # Dummy joint
        tfJointInPrevJoint[1] = np.matrix( [ [     1,     0,     0,     0],
                                             [     0,     1,     0,     0],
                                             [     0,     0,     1,  d_1b],
                                             [     0,     0,     0,     1] ] )

        # Rear spine joint
        tfJointInPrevJoint[2] = np.matrix( [ [  c[2], -s[2],     0,     0],
                                             [     0,     0,     1,     0],
                                             [ -s[2], -c[2],     0,     0],
                                             [     0,     0,     0,     1] ] )

        for j in range(0, 3):
            # Assign joint transforms, in preceeding joint coords and in world coords
            self.spine.joints[j].tfJointInPrevJoint = deepcopy(tfJointInPrevJoint[j])
            if j == 0:
                T = self.baseTarget * self.spine.tfSpineBaseInRobotBase
            else:
                T = self.spine.joints[j-1].tfJointInWorld
            self.spine.joints[j].tfJointInWorld = T * tfJointInPrevJoint[j]


    def moveBase(self):
        # Update spine (FK)
        self.runSpineFK()
        # Update legs (IK)
        for i in range(0, len(self.legs)):
            self.runLegIK(i)


    def runLegFK(self, legIndex):
        leg = self.legs[legIndex]

        s = [0, 0, 0, 0, 0, 0]  # Last value is unused
        c = [0, 0, 0, 0, 0, 0]
        for i in range(0, 5):
            s[i] = math.sin( math.radians(leg.angles[i]) )
            c[i] = math.cos( math.radians(leg.angles[i]) )

        tfJointInPrevJoint = [0, 0, 0, 0, 0, 0]

        tfJointInPrevJoint[0] = np.matrix( [ [  c[0], -s[0],     0,  self.a[0]],
                                             [  s[0],  c[0],     0,          0],
                                             [     0,     0,     1,          0],
                                             [     0,     0,     0,          1] ] )

        tfJointInPrevJoint[1] = np.matrix( [ [  c[1], -s[1],     0,  self.a[1]],
                                             [     0,     0,    -1,          0],
                                             [  s[1],  c[1],     0,          0],
                                             [     0,     0,     0,          1] ] )

        tfJointInPrevJoint[2] = np.matrix( [ [  c[2], -s[2],     0,  self.a[2]],
                                             [  s[2],  c[2],     0,          0],
                                             [     0,     0,     1,          0],
                                             [     0,     0,     0,          1] ] )

        tfJointInPrevJoint[3] = np.matrix( [ [  c[3], -s[3],     0,  self.a[3]],
                                             [  s[3],  c[3],     0,          0],
                                             [     0,     0,     1,          0],
                                             [     0,     0,     0,          1] ] )

        tfJointInPrevJoint[4] = np.matrix( [ [  c[4], -s[4],     0,  self.a[4]],
                                             [     0,     0,     1,          0],
                                             [ -s[4], -c[4],     1,          0],
                                             [     0,     0,     0,          1] ] )

        tfJointInPrevJoint[5] = np.matrix( [ [     1,     0,     0,  self.a[5]],
                                             [     0,     1,     0,          0],
                                             [     0,     0,     1,          0],
                                             [     0,     0,     0,          1] ] )

        for j in range(0, 6):
            # Assign joint transforms, in preceeding joint coords and in world coords
            leg.joints[j].tfJointInPrevJoint = deepcopy(tfJointInPrevJoint[j])
            if j == 0:
                if (leg.id == "FL") or (leg.id == "FR"):
                    T = self.spine.joints[0].tfJointInWorld * leg.tfLegBaseInSpineBase
                else:
                    T = self.spine.joints[2].tfJointInWorld * leg.tfLegBaseInSpineBase
            else:
                T = leg.joints[j-1].tfJointInWorld
            leg.joints[j].tfJointInWorld = T * tfJointInPrevJoint[j]


    def runLegIK(self, legIndex):
        target = self.legTargets[legIndex]
        # Convert target in world to be in leg base
        leg = self.legs[legIndex]
        tfSpineBaseInLegBase = np.linalg.inv(leg.tfLegBaseInSpineBase)
        if (leg.id == "FL") or (leg.id == "FR"):
            worldInSpineBase = np.linalg.inv(self.spine.joints[0].tfJointInWorld)
        else:
            worldInSpineBase = np.linalg.inv(self.spine.joints[2].tfJointInWorld)
        targetInLegBase = tfSpineBaseInLegBase * worldInSpineBase * target

        Tx = targetInLegBase[0, 3]
        Ty = targetInLegBase[1, 3]
        Tz = targetInLegBase[2, 3]

        # Extract roll/pitch/yaw (as seen in World frame) from rotation matrix
        roll, pitch, yaw = getRollPitchYaw(target)

        # Trig. values
        sr = math.sin(roll)
        cr = math.cos(roll)
        sp = math.sin(pitch)
        cp = math.cos(pitch)


        # Foot link projected onto axes
        #aFx = self.a[5]*cr*cp
        #aFy = self.a[5]*sr
        #aFz = self.a[5]*sp

        s2r = math.pow(sr, 2)
        c2r = math.pow(cr, 2)
        s2p = math.pow(sp, 2)
        c2p = math.pow(cp, 2)

        aFSq = math.pow(self.a[5], 2)

        eps = 1e-10
        if abs(roll) < eps:
            aFx = self.a[5]*cp
            aFy = 0
            aFz = self.a[5]*sp
        elif abs(pitch) < eps:
            aFx = self.a[5]*cr
            aFy = self.a[5]*sr
            aFz = 0
        elif abs( abs(roll) - math.pi/2 ) < eps:
            aFx = 0
            aFy = np.sign(roll)*self.a[5]
            aFz = 0
        elif abs( abs(pitch) - math.pi/2 ) < eps:
            aFx = 0
            aFy = self.a[5]*sr
            aFz = self.a[5]*cr
        else:
            A = math.sqrt( aFSq * c2p / (1 - s2r*s2p) )
            C = A*cr/cp

            aFx = A*cr
            aFy = A*sr
            aFz = C*sp

            #B = math.sqrt( math.pow(aFy, 2) + math.pow(aFz, 2) )


        # Solve Joint 1
        num = Ty + aFy
        den = Tx - aFx
        a0 = math.atan2(num, den)
        leg.angles[0] = math.degrees(a0)

        # Leg links projected onto z-axis
        c0 = math.cos(a0)
        a2z = self.a[1]*c0
        a3z = self.a[2]*c0
        a4z = self.a[3]*c0
        a5z = self.a[4]*c0*sp

        # Additional vars
        a5x = self.a[4]*c0*cp
        X = a5x + aFx
        Z = a5z + aFz
        j4Height = Tx - a2z - X
        j2j4DistSquared = math.pow(j4Height, 2) + math.pow(Tz + Z, 2)
        j2j4Dist = math.sqrt(j2j4DistSquared)

        # Solve Joint 2
        num = Tz + Z
        den = j4Height
        psi = math.degrees( math.atan2(num, den) )

        num = math.pow(a3z, 2) + j2j4DistSquared - math.pow(a4z, 2)
        den = 2.0*a3z*j2j4Dist
        if abs(num) <= abs(den):
            phi = math.degrees( math.acos(num/den) )
            leg.angles[1] = - (phi - psi)

        # Solve Joint 3
        num = math.pow(a3z, 2) + math.pow(a4z, 2) - j2j4DistSquared
        den = 2.0*a3z*a4z
        if abs(num) <= abs(den):
            leg.angles[2] = 180.0 - math.degrees( math.acos(num/den) )

        # Solve Joint 4
        num = math.pow(a4z, 2) + j2j4DistSquared - math.pow(a3z, 2)
        den = 2.0*a4z*j2j4Dist
        if abs(num) <= abs(den):
            omega = math.degrees( math.acos(num/den) )
            leg.angles[3] = - ( psi + omega + math.degrees(pitch) )

        # Solve Joint 5
        leg.angles[4] = - math.degrees(a0 + roll)

        self.runLegFK(legIndex)

        #print("target: ", target)
        #print("targetInLegBase: ", targetInLegBase)
        #print("leg.angles: ", leg.angles)


    def testIKStep(self, t):
        aEll = 60
        bEll = 20
        xAdjust = 0
        yAdjust = 30
        u = math.tan(t/2.0)
        u2 = math.pow(u, 2)
        x = aEll*(1 - u2) / (u2 + 1)
        y = 2*bEll*u / (u2 + 1)
        self.legTargets[self.selectedLeg][0, 3] = self.legTargetsHome[self.selectedLeg][0, 3] + x + xAdjust
        self.legTargets[self.selectedLeg][2, 3] = self.legTargetsHome[self.selectedLeg][2, 3] + y + yAdjust
        self.runLegIK(self.selectedLeg)


class Spine():
    def __init__(self, id, joints, angles, tfSpineBaseInRobotBase):
        self.id = id
        self.joints = joints
        self.angles = angles
        self.tfSpineBaseInRobotBase = tfSpineBaseInRobotBase


class Leg():
    def __init__(self, id, joints, angles, tfLegBaseInSpineBase):
        self.id = id
        self.joints = joints
        self.angles = angles
        self.tfLegBaseInSpineBase = tfLegBaseInSpineBase


class Joint():
    def __init__(self, id, tfJointInPrevJoint, tfJointInWorld):
        self.id = id
        self.tfJointInPrevJoint = tfJointInPrevJoint
        self.tfJointInWorld = tfJointInWorld


def initSpine():
    tfSpineBaseInRobotBase = identityTF()

    # TODO: Get this translation accurate e.g. at location of IMU
    tfSpineBaseInRobotBase *= np.matrix( [ [  1,  0,  0, -50],
                                           [  0,  1,  0,   0],
                                           [  0,  0,  1,   0],
                                           [  0,  0,  0,   1] ] )

    # -45 around Y (to get from world to robot spine)
    s = math.sin( -math.pi/4 )
    c = math.cos( -math.pi/4 )
    tfSpineBaseInRobotBase *= np.matrix( [ [  c,  0,  s,   0],
                                           [  0,  1,  0,   0],
                                           [ -s,  0,  c,   0],
                                           [  0,  0,  0,   1] ] )

    spineAngles = [0, 0, 0]
    spine = Spine( "B", initSpineJoints(21), spineAngles, tfSpineBaseInRobotBase )
    return spine


def initSpineJoints(startingJoint):
    tmpTF = identityTF()
    joints = [0, 0, 0]
    joints[0] = Joint(startingJoint, tmpTF, tmpTF)
    joints[1] = Joint("Dummy", tmpTF, tmpTF)
    joints[2] = Joint(startingJoint + 1, tmpTF, tmpTF)
    return joints


def initLegs():
    # TODO: Position leg bases more accurately
    lengthD = 100
    widthD = 50
    heightD = 10

    # +135 around Y
    s = math.sin( 3*math.pi/4 )
    c = math.cos( 3*math.pi/4 )
    tfFLBaseInSpineBase = np.matrix( [ [  c,  0,  s,  0],
                                       [  0,  1,  0,  0],
                                       [ -s,  0,  c,  0],
                                       [  0,  0,  0,  1] ] )
    tfFLBaseInSpineBase *= np.matrix( [ [  1,  0,  0, -heightD],
                                        [  0,  1,  0,   widthD],
                                        [  0,  0,  1,  lengthD],
                                        [  0,  0,  0,        1] ] )
    # +135 around Y
    # c, s same as above
    tfFRBaseInSpineBase = np.matrix( [ [  c,  0,  s,  0],
                                       [  0,  1,  0,  0],
                                       [ -s,  0,  c,  0],
                                       [  0,  0,  0,  1] ] )
    tfFRBaseInSpineBase *= np.matrix( [ [  1,  0,  0, -heightD],
                                        [  0,  1,  0,  -widthD],
                                        [  0,  0,  1,  lengthD],
                                        [  0,  0,  0,        1] ] )

    # +90 around X
    s = math.sin( math.pi/2 )
    c = math.cos( math.pi/2 )
    T = np.matrix( [ [  1,  0,  0,  0],
                     [  0,  c, -s,  0],
                     [  0,  s,  c,  0],
                     [  0,  0,  0,  1] ] )
    # +180 around Y
    s = math.sin( math.pi )
    c = math.cos( math.pi )
    tfRLBaseInSpineBase = T * np.matrix( [ [  c,  0,  s,  0],
                                           [  0,  1,  0,  0],
                                           [ -s,  0,  c,  0],
                                           [  0,  0,  0,  1] ] )
    tfRLBaseInSpineBase *= np.matrix( [ [  1,  0,  0,        0],
                                        [  0,  1,  0,   widthD],
                                        [  0,  0,  1, -lengthD],
                                        [  0,  0,  0,        1] ] )
    # +180 around Y
    # c, s same as above
    tfRRBaseInSpineBase = T * np.matrix( [ [  c,  0,  s,  0],
                                           [  0,  1,  0,  0],
                                           [ -s,  0,  c,  0],
                                           [  0,  0,  0,  1] ] )
    tfRRBaseInSpineBase *= np.matrix( [ [  1,  0,  0,        0],
                                        [  0,  1,  0,  -widthD],
                                        [  0,  0,  1, -lengthD],
                                        [  0,  0,  0,        1] ] )

    legs = [0, 0, 0, 0]
    angles = [0, 0, 0, 0, 0]
    sj = 1
    legs[0] = Leg( "FL", initLegJoints(sj), angles, tfFLBaseInSpineBase )
    sj += 5
    legs[1] = Leg( "FR", initLegJoints(sj), angles, tfFRBaseInSpineBase )
    sj += 5
    legs[2] = Leg( "RL", initLegJoints(sj), angles, tfRLBaseInSpineBase )
    sj += 5
    legs[3] = Leg( "RR", initLegJoints(sj), angles, tfRRBaseInSpineBase )

    return legs


def initLegJoints(startingJoint):
    tmpTF = identityTF()
    joints = [0, 0, 0, 0, 0, 0]
    for j in range(0, 5):
        joints[j] = Joint(startingJoint + j, tmpTF, tmpTF)
    joints[5] = Joint("F", tmpTF, tmpTF)  # Foot
    return joints
