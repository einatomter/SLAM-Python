import math
import numpy as np
from fractions import Fraction
from scipy.odr import *

# landmarks
LANDMARKS = []



class featuresDetection:
    def __init__(self):
        # variables
        self.EPSILON = 10
        self.DELTA = 501
        self.SNUM = 6
        self.PMIN = 20
        self.GMAX = 20
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERPOINTS) - 1 # total number of laser points
        self.LMIN = 20                      # minimum length of a line segment
        self.LR = 0                         # real length of a line segment
        self.PR = 0                         # the number of laser points contained in the line segment
        self.FEATURES = []

    # euclidian distance from point1 to point2
    @staticmethod
    def distP2P(point1, point2):
        px = (point1[0] - point2[0]) ** 2
        py = (point1[1] - point2[1]) ** 2
        return math.sqrt(px + py)

    # distance point to line written in the general form
    def distP2L(self, params, point):
        A, B, C = params
        distance = abs(A * point[0] + B * point[1] + C) / math.sqrt(A ** 2 + B ** 2)
        return distance

    # extract two points from a line equation under the slope intercepts form
    def lineGet2P(self, m, b):
        x = 5           # chosen arbitrarily
        y = m * x + b
        x2 = 2000       # chosen arbitrarily
        y2 = m * x2 + b
        return [(x, y), (x2, y2)]

    # general form to slope-intercept form
    def lineFormG2SI(self, A, B, C):
        m = - A/B
        b = - C/B
        return m, b

    # slope-intercept form to general form
    def lineFormSI2G(self, m, b):
        A, B, C = -m, 1, -b
        if A < 0:
            A, B, C = -A, -B, -C
    
        denA = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        denC = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(denA, denC)
        lcm = denA * denC / gcd

        A = A * lcm
        B = B * lcm
        C = C * lcm

        return A, B, C

    # euclidean coordinates of 2 intersecting lines
    # Note: the formula assumes that the lines will intersect
    def lineIntersectGeneral(self, params1, params2):
        A1, B1, C1 = params1
        A2, B2, C2 = params2

        x = (C1 * B2 - B1 * C2) / (B1 * A2 - A1 * B2)
        y = (A1 * C2 - A2 * C1) / (B1 * A2 - A1 * B2)
        return x, y

    # slope-intercept form from 2 points
    def lineFrom2P(self, point1, point2):
        m, b = 0, 0

        # if x values are the same, result is vertical line which cannot be represented
        if point1[0] == point2[0]:
            # print('warning: points form vertical line')
            pass
        else:
            m = (point2[1] - point1[1]) / (point2[0] - point1[0])
            b = point2[1] - m * point2[0]

        return m, b
    
    # euclidian coordinates of point projected onto a line
    def projectionP2L(self, point, m, b):
        x, y  = point
        m2 = -1 / m
        c2 = y - m2 * x
        intersectionx = - (b - c2) / (m - m2)
        intersectiony = m2 * intersectionx + c2
        return intersectionx, intersectiony

    # r, theta to x, y
    def ad2pos(self, distance, angle, robotPosition):
        x = distance * math.cos(angle) + robotPosition[0]
        y = -distance * math.sin(angle) + robotPosition[1]
        return (int(x), int(y))

    #
    def laserPointsSet(self, data):
        self.LASERPOINTS = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = self.ad2pos(point[0], point[1], point[2])
                self.LASERPOINTS.append([coordinates, point[1]])

        self.NP = len(self.LASERPOINTS) - 1 # to avoid overflow

    # define a function (linear in our case) to fit the data with
    def linearFunc(self, p, x):
        m, b = p
        return m * x + b

    # create a fitted model using ODR
    def odrFit(self, laserPoints):
        x = np.array([i[0][0] for i in laserPoints])
        y = np.array([i[0][1] for i in laserPoints])

        # create a model for fitting
        linearModel = Model(self.linearFunc)

        # create a RealData object using our initiated data from above
        data = RealData(x, y)

        # set up ODR with the model and data
        odrModel = ODR(data, linearModel, beta0=[0., 0.])

        # run the regression
        out = odrModel.run()
        m, b = out.beta
        return m, b

    #
    def predictPoint(self, lineParams, sensedPoint, robotPos):
        m, b = self.lineFrom2P(robotPos, sensedPoint)
        params1 = self.lineFormSI2G(m, b)
        predx, predy = self.lineIntersectGeneral(params1, lineParams)

        return predx, predy

    def seedSegmentDetection(self, robotPos, breakPointInd):
        flag = True
        self.NP = max(0, self.NP)
        self.SEED_SEGMENTS = []

        for i in range(breakPointInd, (self.NP - self.PMIN)):
            predictedPointsToDraw = []
            j = i + self.SNUM
            m, c = self.odrFit(self.LASERPOINTS[i:j])

            params = self.lineFormSI2G(m, c)

            # iterate over points to see if it should be in the line segment
            for k in range(i, j):
                predictedPoint = self.predictPoint(params, self.LASERPOINTS[k][0], robotPos)
                predictedPointsToDraw.append(predictedPoint)

                d1 = self.distP2P(predictedPoint, self.LASERPOINTS[k][0])
                if d1 > self.DELTA:
                    flag = False
                    break

                # TODO: check if 2nd param should be predictedPoint or self.LASERPOINTS[k][0]
                d2 = self.distP2L(params, self.LASERPOINTS[k][0])
                if d2 > self.EPSILON:
                    flag = False
                    break

            if flag:
                self.LINE_PARAMS = params
                return [self.LASERPOINTS[i:j], predictedPointsToDraw, (i, j)]

        return False

    def seedSegmentGrowing(self, indices, breakPoint):
        lineEq = self.LINE_PARAMS
        i, j = indices

        # beginning and final points in the line segment
        PB, PF = max(breakPoint, i - 1), min(j + 1, len(self.LASERPOINTS) - 1)

        # grow seed segment on left side
        while self.distP2L(lineEq, self.LASERPOINTS[PF][0]) < self.EPSILON:
            if PF > self.NP - 1:
                break
            else:
                m, b = self.odrFit(self.LASERPOINTS[PB:PF])
                lineEq = self.lineFormSI2G(m, b)
                POINT = self.LASERPOINTS[PF][0]

            PF = PF + 1
            NEXTPOINT = self.LASERPOINTS[PF][0]
            if self.distP2P(POINT, NEXTPOINT) > self.GMAX:
                break

        PF = PF - 1

        # grow seed segment on right side
        while self.distP2L(lineEq, self.LASERPOINTS[PB][0]) < self.EPSILON:
            if PB < breakPoint:
                break
            else:
                m, b = self.odrFit(self.LASERPOINTS[PB:PF])
                lineEq = self.lineFormSI2G(m, b)
                POINT = self.LASERPOINTS[PB][0]

            PB = PB - 1
            NEXTPOINT = self.LASERPOINTS[PB][0]
            if self.distP2P(POINT, NEXTPOINT) > self.GMAX:
                break

        PB = PB + 1

        LR = self.distP2P(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
        PR = len(self.LASERPOINTS[PB:PF])

        if (LR >= self.LMIN) and (PR >= self.PMIN):
            self.LINE_PARAMS = lineEq
            m, b = self.lineFormG2SI(lineEq[0], lineEq[1], lineEq[2])
            self.twoPoints = self.lineGet2P(m, b)
            self.LINE_SEGMENTS.append((self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]))
            return [self.LASERPOINTS[PB:PF], self.twoPoints,
                   (self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]), PF, lineEq, (m,b)]
        else:
            return False

    # convert line feature to a point representation
    def lineF2P(self):
        newRep = [] # the new representation of the features

        for feature in self.FEATURES:
            projection = self.projectionP2L((0,0), feature[0][0], feature[0][1])
            newRep.append([feature[0], feature[1], projection])

        return newRep

def isOverlap(seg1, seg2):
    length1 = featuresDetection.distP2P(seg1[0], seg1[1])
    length2 = featuresDetection.distP2P(seg2[0], seg2[1])
    center1 = ((seg1[0][0] + seg1[1][0])/2, (seg1[0][1] + seg1[1][1])/2)
    center2 = ((seg2[0][0] + seg2[1][0])/2, (seg2[0][1] + seg2[1][1])/2)

    dist = featuresDetection.distP2P(center1, center2)
    if dist > (length1 + length2)/2:
        return False
    else:
        return True

def landmarkAssociation(landmarks):
    thresh = 10
    for l in landmarks:

        flag = False
        for i, landmark in enumerate(LANDMARKS):
            dist = featuresDetection.distP2P(l[2], landmark[2])
            if dist < thresh:
                if not isOverlap(l[1], landmark[1]):
                    continue
                else:
                    LANDMARKS.pop(i)
                    LANDMARKS.insert(i, l)
                    flag = True
                    break
        if not flag:
            LANDMARKS.append(l)
