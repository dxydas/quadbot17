from HelperFunctions import identityTF, applyYawPitchRoll

import numpy as np
import math
from copy import deepcopy


# Link lengths "a-1" (last value is the foot offset)
a = [0, 29.05, 76.919, 72.96, 45.032, 33.596]

# Offsets for natural "home" position
spineAngleOffsets = [0, 0, -45]
legAngleOffsets = [0, -34, 67.5, -33.5, 0]


def runSpineFK(spine, legs, targets, x, y, z, roll, pitch, yaw):
    # Spine front: In the future this can be controlled by e.g. orientation from IMU
    spine.tfSpineBaseInWorld = identityTF()

    spine.tfSpineBaseInWorld[0, 3] = x
    spine.tfSpineBaseInWorld[1, 3] = y
    spine.tfSpineBaseInWorld[2, 3] = z

    applyYawPitchRoll(spine.tfSpineBaseInWorld, yaw, pitch, roll)

    # TODO: Get this translation accurate e.g. at location of IMU
    # Translation (to get from world to robot spine)
    spine.tfSpineBaseInWorld *= np.matrix( [ [  1,  0,  0, -50],
                                             [  0,  1,  0,   0],
                                             [  0,  0,  1,   0],
                                             [  0,  0,  0,   1] ] )

    # -45 around Y (to get from world to robot spine)
    s = math.sin( -math.pi/4 )
    c = math.cos( -math.pi/4 )
    spine.tfSpineBaseInWorld *= np.matrix( [ [  c,  0,  s,   0],
                                             [  0,  1,  0,   0],
                                             [ -s,  0,  c,   0],
                                             [  0,  0,  0,   1] ] )

    d_1b = 16.975  # Dummy link offset

    s = [0, 0, 0, 0]
    c = [0, 0, 0, 0]
    for i in range(1, 4):
        s[i] = math.sin( math.radians(spine.angles[i-1]) )
        c[i] = math.cos( math.radians(spine.angles[i-1]) )

    tfJointInPrevJoint = [0, 0, 0]

    # Front spine joint
    tfJointInPrevJoint[0] = np.matrix( [ [  c[1], -s[1],     0,     0],
                                         [  s[1],  c[1],     0,     0],
                                         [     0,     0,     1,     0],
                                         [     0,     0,     0,     1] ] )

    # Dummy joint
    tfJointInPrevJoint[1] = np.matrix( [ [     1,     0,     0,     0],
                                         [     0,     1,     0,     0],
                                         [     0,     0,     1,  d_1b],
                                         [     0,     0,     0,     1] ] )

    # Rear spine joint
    tfJointInPrevJoint[2] = np.matrix( [ [  c[3], -s[3],     0,     0],
                                         [     0,     0,     1,     0],
                                         [ -s[3], -c[3],     0,     0],
                                         [     0,     0,     0,     1] ] )

    for j in range(0, 3):
        # Assign joint transforms, in preceeding joint coords and in world coords
        spine.joints[j].tfJointInPrevJoint = deepcopy(tfJointInPrevJoint[j])
        if j == 0:
            T = spine.tfSpineBaseInWorld
        else:
            T = spine.joints[j-1].tfJointInWorld
        spine.joints[j].tfJointInWorld = T * tfJointInPrevJoint[j]

    # Update legs
    for i, leg in enumerate(legs):
        runLegIK(spine, leg, targets[i])


def runSpineIK():
    #TODO
    pass


def runLegFK(spine, leg):
    s = [0, 0, 0, 0, 0]
    c = [0, 0, 0, 0, 0]
    for i in range(0, 5):
        s[i] = math.sin( math.radians(leg.angles[i]) )
        c[i] = math.cos( math.radians(leg.angles[i]) )

    tfJointInPrevJoint = [0, 0, 0, 0, 0, 0]

    tfJointInPrevJoint[0] = np.matrix( [ [  c[0], -s[0],     0,  a[0]],
                                         [  s[0],  c[0],     0,     0],
                                         [     0,     0,     1,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[1] = np.matrix( [ [  c[1], -s[1],     0,  a[1]],
                                         [     0,     0,    -1,     0],
                                         [  s[1],  c[1],     0,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[2] = np.matrix( [ [  c[2], -s[2],     0,  a[2]],
                                         [  s[2],  c[2],     0,     0],
                                         [     0,     0,     1,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[3] = np.matrix( [ [  c[3], -s[3],     0,  a[3]],
                                         [  s[3],  c[3],     0,     0],
                                         [     0,     0,     1,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[4] = np.matrix( [ [  c[4], -s[4],     0,  a[4]],
                                         [     0,     0,     1,     0],
                                         [ -s[4], -c[4],     1,     0],
                                         [     0,     0,     0,     1] ] )

    tfJointInPrevJoint[5] = np.matrix( [ [  1,  0,  0, a[5]],
                                         [  0,  1,  0,    0],
                                         [  0,  0,  1,    0],
                                         [  0,  0,  0,    1] ] )

    for j in range(0, 6):
        # Assign joint transforms, in preceeding joint coords and in world coords
        leg.joints[j].tfJointInPrevJoint = deepcopy(tfJointInPrevJoint[j])
        if j == 0:
            if (leg.id == "FL") or (leg.id == "FR"):
                T = spine.tfSpineBaseInWorld * leg.tfLegBaseInSpineBase
            else:
                T = spine.joints[2].tfJointInWorld * leg.tfLegBaseInSpineBase
        else:
            T = leg.joints[j-1].tfJointInWorld
        leg.joints[j].tfJointInWorld = T * tfJointInPrevJoint[j]


def runLegIK(spine, leg, target):
    # Convert target in world to be in leg base
    tfSpineBaseInLegBase = np.linalg.inv(leg.tfLegBaseInSpineBase)
    if (leg.id == "FL") or (leg.id == "FR"):
        T = spine.tfSpineBaseInWorld
        worldInSpineBase = np.linalg.inv(spine.tfSpineBaseInWorld)
    else:
        worldInSpineBase = np.linalg.inv(spine.joints[2].tfJointInWorld)
    targetInLegBase = tfSpineBaseInLegBase * worldInSpineBase * target

    Tx = targetInLegBase[0, 3]
    Ty = targetInLegBase[1, 3]
    Tz = targetInLegBase[2, 3]

    # Extract roll/pitch/yaw (as seen in World frame) from rotation matrix
    r11 = target[0, 0]
    r21 = target[1, 0]
    r31 = target[2, 0]
    r32 = target[2, 1]
    r33 = target[2, 2]
    den = math.sqrt( math.pow(r32, 2) + math.pow(r33, 2) )
    # Roll negated
    roll = - math.atan2( r32, r33 )
    pitch = math.atan2( -r31, den )
    yaw = math.atan2( r21, r11 )

    # Trig. values
    sr = math.sin(roll)
    cr = math.cos(roll)
    sp = math.sin(pitch)
    cp = math.cos(pitch)


    # Foot link projected onto axes
    #aFx = a[5]*cr*cp
    #aFy = a[5]*sr
    #aFz = a[5]*sp

    s2r = math.pow(sr, 2)
    c2r = math.pow(cr, 2)
    s2p = math.pow(sp, 2)
    c2p = math.pow(cp, 2)

    aFSq = math.pow(a[5], 2)

    eps = 1e-10
    if abs(roll) < eps:
        aFx = a[5]*cp
        aFy = 0
        aFz = a[5]*sp
    elif abs(pitch) < eps:
        aFx = a[5]*cr
        aFy = a[5]*sr
        aFz = 0
    elif abs( abs(roll) - math.pi/2 ) < eps:
        aFx = 0
        aFy = np.sign(roll)*a[5]
        aFz = 0
    elif abs( abs(pitch) - math.pi/2 ) < eps:
        aFx = 0
        aFy = a[5]*sr
        aFz = a[5]*cr
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
    a2z = a[1]*c0
    a3z = a[2]*c0
    a4z = a[3]*c0
    a5z = a[4]*c0*sp

    # Additional vars
    a5x = a[4]*c0*cp
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

    runLegFK(spine, leg)

    #print "target: ", target
    #print "targetInLegBase: ", targetInLegBase
    #print "leg.angles: ", leg.angles
