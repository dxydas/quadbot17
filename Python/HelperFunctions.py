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
    return (old - oldMin) * newRange / oldRange + newMin


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
