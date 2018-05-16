import numpy as np
import math


def identityTF():
    return np.matrix( [ [ 1.0,   0,   0,   0],
                        [   0, 1.0,   0,   0],
                        [   0,   0, 1.0,   0],
                        [   0,   0,   0, 1.0] ] )


def rescale(old, oldMin, oldMax, newMin, newMax):
    oldRange = (oldMax - oldMin)
    newRange = (newMax - newMin)
    if old < oldMin:
        old = oldMin
    if old > oldMax:
        old = oldMax
    return (old - oldMin) * newRange / oldRange + newMin


def getRollPitchYaw(T):
    r11 = T[0, 0]
    r21 = T[1, 0]
    r31 = T[2, 0]
    r32 = T[2, 1]
    r33 = T[2, 2]
    den = math.sqrt( math.pow(r32, 2) + math.pow(r33, 2) )
    # Roll is negated
    roll = - math.atan2( r32, r33 )
    pitch = math.atan2( -r31, den )
    yaw = math.atan2( r21, r11 )
    return roll, pitch, yaw


def applyYawPitchRoll(T, yaw, pitch, roll):
    s = math.sin( math.radians(yaw) )
    c = math.cos( math.radians(yaw) )
    Rot = np.matrix( [ [  c, -s,  0,  0],
                       [  s,  c,  0,  0],
                       [  0,  0,  1,  0],
                       [  0,  0,  0,  1] ] )
    s = math.sin( math.radians(pitch) )
    c = math.cos( math.radians(pitch) )
    Rot *= np.matrix( [ [  c,  0,  s,  0],
                        [  0,  1,  0,  0],
                        [ -s,  0,  c,  0],
                        [  0,  0,  0,  1] ] )
    s = math.sin( math.radians(roll) )
    c = math.cos( math.radians(roll) )
    Rot *= np.matrix( [ [  1,  0,  0,  0],
                        [  0,  c, -s,  0],
                        [  0,  s,  c,  0],
                        [  0,  0,  0,  1] ] )
    for r in range(0, 3):
        for c in range(0, 3):
            T[r, c] = Rot[r, c]
