#!/usr/bin/env python

import math

zeroPositions = [850, 320, 512, 1770, 1500, 650]

analogLowerSignalBound = 500
analogUpperSignalBound = 2500
digitalLowerSignalBound = 0
digitalUpperSignalBound = 1024

lowerSignalBound = [
    digitalLowerSignalBound,
    digitalLowerSignalBound,
    digitalLowerSignalBound,
    analogLowerSignalBound,
    analogLowerSignalBound,
    analogLowerSignalBound,
]
upperSignalBound = [
    digitalUpperSignalBound,
    digitalUpperSignalBound,
    digitalUpperSignalBound,
    analogUpperSignalBound,
    analogUpperSignalBound,
    analogUpperSignalBound,
]

digitalScaleFactor = 1024 / ((330.0 / 2.0) * math.pi / 180.0)
analogScaleFactor = 2000 / (180.0 * math.pi / 180.0)

scalingFactors = [
    digitalScaleFactor,
    digitalScaleFactor,
    digitalScaleFactor / 2.0,
    analogScaleFactor,
    analogScaleFactor,
    analogScaleFactor,
]


for i in range(0, 6):
    lowerAngle = (lowerSignalBound[i] - zeroPositions[i]) / scalingFactors[i]
    upperAngle = (upperSignalBound[i] - zeroPositions[i]) / scalingFactors[i]
    print(str(i) + " " + str(lowerAngle) + " " + str(upperAngle))
