import numpy as np
#import matplotlib.pyplot as plt
#from roverbot_lib import *

ROBOT_RADIUS = 0.15/2
OBS_RADIUS = 0.15/2#2*ROBOT_RADIUS+0.2

class Navigation:

    def __init__(self, fov=62):
        self.fov = fov+1
        self.attract = np.zeros([fov, ], dtype='float')
        self.repulse = np.zeros([fov, ], dtype='float')
        self.result = np.zeros([fov, ], dtype='float')
        self.optTraj = int(fov/2)

    def Navigate(self, goalPos):
        self.CreateAtrractionField(goalPos)
        self.optTraj = np.argmax(self.attract)
        vel, rVel = self.CalcVelocities(goalPos)
        return vel, rVel

    def CreateAtrractionField(self, goalPos):
        if goalPos == None:
            self.repulse = np.zeros([self.fov, ], dtype='float')+0.5
            return
        self.attract = np.arange(self.fov, dtype='float')

        goalPos = int(goalPos)#+30 ##remove for non sim and change to deg
        #print(goalPos)
        self.attract[goalPos] = 1

        gradient = 1 / max(goalPos, (self.fov-1) - goalPos)

        a = (self.attract[:goalPos] - goalPos)
        b = (self.attract[goalPos + 1:] - goalPos)
        self.attract[:goalPos] = a * gradient + 1
        self.attract[goalPos + 1:] = b * -gradient + 1
 

    def CalcVelocities(self, goalPos):
        maxVel = 0.2
        maxRot = 0.4
        trajectoryError = (self.fov/2)-self.optTraj
        #print(self.optTraj)
        #print(trajectoryError)
        #print(f"Trajectory: {trajectoryError} with optimal traj: {self.optTraj}")
        rotationalV = min(maxRot, max(0, np.deg2rad(abs(trajectoryError))*0.5))
        #rotationalV = min(maxRot, maxRot - max(0, maxRot - np.deg2rad(abs(trajectoryError)) * 0.5))
        linarV = maxVel * (1 - 0.8*abs(np.deg2rad(rotationalV))/maxRot)
        print(trajectoryError)
        if trajectoryError > 0:
            rotationalV = rotationalV*-1
        rotationalV = round(rotationalV,2)
        linarV = round(linarV, 2)

        return linarV, rotationalV


def main():
    ggmp = Navigation(62)
    vel, rVel = ggmp.Navigate(45, [[0.5, 30], [0.8, 55]])
    a = 2


if __name__ == "__main__":
    main()
