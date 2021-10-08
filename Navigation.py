import numpy as np
import matplotlib.pyplot as plt
from roverbot_lib import *

ROBOT_RADIUS = 0.15/2
OBS_RADIUS = 0.15/2#2*ROBOT_RADIUS+0.2

class Navigation:

    def __init__(self, fov=62):
        self.fov = fov
        self.attract = np.zeros([fov, ], dtype='float')
        self.repulse = np.zeros([fov, ], dtype='float')
        self.result = np.zeros([fov, ], dtype='float')
        self.optTraj = int(fov/2)

    def Navigate(self, goalPos, obstaclePos):
        #goalPos = [int(x) for x in goalPos]
        self.CreateAtrractionField(goalPos[1])
        #self.CreateRepulseField(obstaclePos)
        self.CalcResidualField()
        vel, rVel = self.CalcVelocities(goalPos)
        #print(f"{vel} and {rVel}\n")
        return vel, rVel

    def CreateAtrractionField(self, goalPos):
        if goalPos == None:
            self.repulse = np.zeros([self.fov, ], dtype='float')+0.5
            return
        self.attract = np.arange(self.fov, dtype='float')

        goalPos = 30 - int(np.rad2deg(goalPos[0][1]))#+30 ##remove for non sim and change to deg
        self.attract[goalPos] = 1

        gradient = 1 / max(goalPos, self.fov - goalPos)

        a = (self.attract[:goalPos] - goalPos)
        b = (self.attract[goalPos + 1:] - goalPos)
        self.attract[:goalPos] = a * gradient + 1
        self.attract[goalPos + 1:] = b * -gradient + 1
        #plot
        '''
        plt.figure(3)
        plt.plot(self.attract)
        plt.xlabel('Angle (Degree)')
        plt.ylabel('Attractiveness')
        plt.show()
        '''

    def CreateRepulseField(self, obstaclePos):
        self.repulse = np.zeros([self.fov, ], dtype='float')
        if obstaclePos == None:
            return
        for obstacle in obstaclePos:
            obsDist = obstacle[0]
            obsBearing = obstacle[1]
            a = OBS_RADIUS
            obs_angle_buff = int(np.rad2deg(np.arcsin(OBS_RADIUS/obsDist)))

            obsEffect = max(0, 1- min(1, (obsDist - ROBOT_RADIUS*2)))
            self.repulse[obsBearing] = obsEffect

            angleLeft = max(0,obsBearing-obs_angle_buff)
            angleRight = min(self.fov,obsBearing+obs_angle_buff)
            for angle in range(angleLeft, obsBearing):
                self.repulse[angle] = max(self.repulse[angle], obsEffect)
            for angle in range(obsBearing+1, angleRight+1):
                self.repulse[angle] = max(self.repulse[angle], obsEffect)

        # plot
        '''
        plt.figure(1)
        plt.plot(self.repulse)
        plt.xlabel('Angle (Degree)')
        plt.ylabel('Repulse')
        plt.show()
        '''

    def CalcResidualField(self):
        self.result = self.attract-self.repulse
        self.result = self.result.clip(min=0)

        # plot
        '''plt.figure(2)
        plt.plot(self.result)
        plt.xlabel('Angle (Degree)')
        plt.ylabel('Attractiveness')
        plt.show()'''

        #find optimal trajectory
        self.optTraj = np.argmax(self.result)

    def CalcVelocities(self, goalPos):
        maxVel = 0.07
        maxRot = 0.2
        trajectoryError = (self.fov/2)-self.optTraj
        print(f"Trajectory: {trajectoryError} with optimal traj: {self.optTraj}")
        #rotationalV = min(maxRot, max(0, np.deg2rad(abs(trajectoryError))*0.3))
        rotationalV = maxRot - max(0, maxRot - np.deg2rad(abs(trajectoryError)) * 0.5)
        linarV = maxVel * (1 - 0.8*abs(np.deg2rad(rotationalV))/maxRot)
        if trajectoryError < 0:
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
