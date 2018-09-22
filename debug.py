#!usr/bin/python

# TODO: evaluate avoid obstacle field for multiple obstacles

import numpy as np
import random
import cv2
import math
import time

from src.un_field import univectorField


LEFT = 0
RIGHT = 1

SIMULATION = 0 # turn on simulation
              # if you turn on the simulantion please make the RIGHTectories: erros and erros/log
EPOCH = 1 # how many simulations (min value is one)


w, h = 150, 130 # the width and height sizes of the field in centimiters
teamColor = (255, 0 , 0)
enemyColor = (0, 255, 255)
ballColor = (31, 136, 246)
pathColor = (255, 0, 255)

globalBallPos = None

RADIUS = 4.0
KR = 4.9
K0 = 0.12
DMIN = 5.0
LDELTA = 4.5

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
        color = teamColor

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

            s = cm2pixel(np.array([c, l]))
            new = cm2pixel(np.array(pos)) + 9*v

            new[1] = -new[1]
            cv2.arrowedLine(img, tuple(np.int0(s)), tuple(np.int0(new)), (0,255,255), 1)

def drawPath(img, start, end, univetField):
    currentPos = start
    _currentPos = cm2pixel(currentPos)

    newPos = None
    alpha = 0.8
    beta = 1

    t0 = time.time()

    while(np.linalg.norm(currentPos - end) >= beta):
        theta = univetField.getVec(_robotPos=currentPos, _vRobot=[0,0])
        v = np.array([math.cos(theta), math.sin(theta)])
        newPos = currentPos + (alpha*v)
        _newPos = cm2pixel(newPos).astype(int)

        cv2.line(img, (_currentPos[0], -_currentPos[1]), (_newPos[0], -_newPos[1]), pathColor, 3)

        cv2.imshow('field', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if (time.time() - t0 > 5):
            return False, newPos

        currentPos = newPos
        _currentPos = _newPos
    return True, None


if __name__ == "__main__":
    imgField = cv2.imread('img/vss-field.jpg')

    rep = EPOCH
    i = 0
    while rep > 0:
        imgField2 = np.copy(imgField)

        robot = getRobot()
        globalBallPos = ball = getBall()

        obstaclesList = [getObstacle(), getObstacle(), getObstacle()]
        print "Posicao dos obstaculos: ", obstaclesList
        # obstacle = np.array(obstaclesList)
        # vObstacle = np.array([[0,0], [0,0], [0,0]])

        obstacle    = np.array([])
        vObstacle   = np.array([])

        # obstacle = np.array([getObstacle()])
        # vObstacle = np.array([[0,0]])

        # Drawing components
        drawRobot(imgField2, robot)
        drawObstacles(imgField2, obstacle)
        drawBall(imgField2, cm2pixel(ball))

        # Creates the univector field
        univetField = univectorField(attack_goal=np.array([150, -65]), _rotation=True)
        univetField.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)

        univetField.updateBall(ball)

        univetField.updateObstacles(obstacle, vObstacle)


        drawField(imgField2, univetField)
        ret, pos = drawPath(imgField2, robot, ball, univetField)
        print "Draw path done!"

        # display the path in the field
        if not SIMULATION:
            cv2.imshow('field', imgField2)
            cv2.waitKey(0)
            break
        else:
            if not ret:
                cv2.imwrite('./erros/Erro-'+ str(i)+'.jpg', imgField2)
                nomeArquivo = './erros/log/Erro-'+str(i)+'.txt'
                arquivo = open(nomeArquivo, 'w+')
                texto = "Obstacles: " + str(obstacle) + '\n'
                texto += "Ball: " + str(ball) + '\n'
                texto += "Robot: " + str(pos) + '\n'
                arquivo.writelines(texto)
                arquivo.close()

            rep -= 1
            print "SIMULATION", i
            i += 1
