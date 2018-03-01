#!usr/bin/python

# TODO: evaluate avoid obstacle field for multiple obstacles

import numpy as np
import random
import matplotlib.pyplot as plt
import math

from un_field import avoidObstacle
from un_field import move2Goal
from un_field import angleWithX
from un_field import univectorField

w, h = 75, 65

def getObstacle():
    return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def getBall():
    return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def getRobot():
    return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def printObstacles(_obstacle, plt):
    for i in range(_obstacle.shape[0]):
        plt.plot(_obstacle[i][0], -obstacle[i][1], 'go')


RADIUS = 0.4
KR = 1.04
K0 = 0.12
DMIN = 1.79
LDELTA = 1.14

if __name__:

    # Create matrix to store the fields
    field = np.zeros((h, w))
    goalField = np.zeros((h, w))

    obstacle = np.array([getObstacle(), getObstacle(), getObstacle(), getObstacle(), getObstacle()])
    vObstacle = np.array([[0,0], [0,0], [0,0], [0,0], [0,0]])
    # obstacle = np.array([getObstacle()])
    # vObstacle = np.array([[0, 0]])
    robot = getRobot()
    ball = getBall()

    # Creates an avoid obstacle field
    avdField = avoidObstacle(obstacle, vObstacle, robot, np.array([0, 0]), K0)

    # Creates a move to goal field
    gField = move2Goal(10.0, 0.8)
    gField.updateOrigin([ball[0], ball[1]])

    # Creates an univector field
    univetField = univectorField()
    univetField.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)
    univetField.updateBall(ball)
    univetField.updateObstacles(obstacle, vObstacle)


    # Get the vectors for each position of the repulsive field
    # and move to goal field
    for l in range(h):
        for c in range(w):
            # field[l][c] = avdField.fi_auf([c, -l])
            field[l][c] = univetField.getVec(_robotPos=[c, -l], _vRobot=[0,0])
            goalField[l][c] = gField.fi_tuf(np.array([c, -l]))
    # X-axis of all vectors
    Ug = np.cos(goalField)
    Ur = np.cos(field)
    # Y-axis of all vectors
    Vg = np.sin(goalField)
    Vr = np.sin(field)

    print "start ploting"

    # -----------------------DEBUG MOVE TO GOAL------------------------
    plt.title('Move to goal Field')
    # prints the ball
    plt.plot(ball[0], -ball[1], 'ro')
    plt.plot(robot[0], -robot[1], 'bo')
    printObstacles(obstacle, plt)
    Q = plt.quiver(Ur, Vr, units='width') # to print the move to goal field uncomment this line

    # -----------------------DEBUG REPULSIVE------------------------
    # plt.title('Repulsive Field')
    # # prints the obstacle
    # plt.plot(obstacle[0], obstacle[1], 'ro')
    # Q = plt.quiver(Ur, Vr, units='width') # to print the repulsive field uncomment this line


    plt.show()
