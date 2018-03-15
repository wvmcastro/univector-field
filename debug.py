#!usr/bin/python

# TODO: evaluate avoid obstacle field for multiple obstacles

import numpy as np
import random
import cv2
import math

from src.un_field import avoidObstacle
from src.un_field import move2Goal
from src.un_field import angleWithX
from src.un_field import univectorField

w, h = 150, 130 # the width and height sizes of the field in centimiters
timeColor = (255, 0 , 0)
enemyColor = (0, 255, 255)
ballColor = (31, 136, 246)
pathColor = (255, 0, 255)

RADIUS = 2.0
KR = 1.04
K0 = 0.12
DMIN = 1.79
LDELTA = 2.28

def getObstacle():
    return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def getBall():
    return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def getRobot():
    return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def printObstacles(_obstacle, plt):
    for i in range(_obstacle.shape[0]):
        plt.plot(_obstacle[i][0], -obstacle[i][1], 'go')

def cm2pixel(pos):
    posArray = np.array(pos)
    return 4*posArray

def drawRobot(img, robotPos, enemy=False):
    if enemy:
        color = enemyColor
    else:
        color = timeColor

    pos = cm2pixel([robotPos[0], -robotPos[1]])
    topLeft = (pos[0]-15, pos[1]-15)
    bottomRight = (pos[0]+15, pos[1]+15)
    cv2.rectangle(img, topLeft, bottomRight, color, -1)

def drawObstacles(img, obstacles):
    if obstacles.size:
        for i in range(obstacles.shape[0]):
            drawRobot(img, obstacles[i], enemy=True)

def drawBall(img, ballPos):
    cv2.circle(img, (ballPos[0], -ballPos[1]), 9, ballColor, -1)

def drawField(img, univetField):
    for l in range(0, h, 3):
        for c in range(0, w, 3):
            pos = [c, -l]
            theta = univetField.getVec(_robotPos=pos, _vRobot=[0,0])

            v = np.array([np.cos(theta), np.sin(theta)])

            # print "origin, pos, vet"
            # print univetField.mv2GoalField.origin, pos, v

            # print [pos[0], -pos[1]], v
            s = cm2pixel(np.array([c, l]))
            new = cm2pixel(np.array(pos)) + 10*v
            # print pos, v, pos + 12*v, "1"
            new[1] = -new[1]

            # print s, v, new, "2"
            cv2.arrowedLine(img, tuple(np.int0(s)), tuple(np.int0(new)), (50,50,50), 1)

def drawPath(img, start, end, univetField):
    currentPos = start
    _currentPos = cm2pixel(currentPos)

    newPos = None
    alpha = 0.7
    beta = 1

    while(np.linalg.norm(currentPos - end) >= beta):
        theta = univetField.getVec(_robotPos=currentPos, _vRobot=[0,0])
        v = np.array([math.cos(theta), math.sin(theta)])
        newPos = currentPos + (alpha*v)
        _newPos = cm2pixel(newPos).astype(int)
        
        cv2.line(img, (_currentPos[0], -_currentPos[1]), (_newPos[0], -_newPos[1]), pathColor, 3)

        cv2.imshow('campo', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        currentPos = newPos
        _currentPos = _newPos


if __name__:
    imgField = cv2.imread('img/vss-field.jpg')

    robot = getRobot()
    ball = getBall()

    obstacle = np.array([getObstacle(), getObstacle(), getObstacle(), getObstacle(), getObstacle(), getObstacle()])
    vObstacle = np.array([[0,0], [0,0], [0,0], [0,0], [0,0], [0,0]])

    # Drawing components
    drawRobot(imgField, robot)
    drawObstacles(imgField, obstacle)
    drawBall(imgField, cm2pixel(ball))

    # Creates the univector field
    univetField = univectorField()
    univetField.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)
    univetField.updateBall(ball)
    univetField.updateObstacles(obstacle, vObstacle)

    
    drawField(imgField, univetField)
    drawPath(imgField, robot, ball, univetField)

    # display the path in the field
    cv2.imshow('campo', imgField)
    cv2.waitKey(0)