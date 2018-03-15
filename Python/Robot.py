from HelperFunctions import identityTF

import math
import numpy as np


class Robot():
    def __init__(self):
        self.spine = initSpine()
        self.legs = initLegs()


class Spine():
    def __init__(self, id, joints, angles, tfSpineBaseInWorld):
        self.id = id
        self.joints = joints
        self.angles = angles
        self.tfSpineBaseInWorld = tfSpineBaseInWorld


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
    tmpTF = identityTF()
    global spine
    spineAngles = [0, 0, 0]
    spine = Spine( "B", initSpineJoints(21), spineAngles, tmpTF )
    return spine


def initSpineJoints(startingJoint):
    tmpTF = identityTF()
    joints = [0, 0, 0]
    joints[0] = Joint(startingJoint, tmpTF, tmpTF)
    joints[1] = Joint("Dummy", tmpTF, tmpTF)
    joints[2] = Joint(startingJoint + 1, tmpTF, tmpTF)
    return joints


def initLegs():
    lengthD = 100
    widthD = 50
    heightD = 10

    # TODO: Position leg bases more accurately

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
