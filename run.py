# import the soccer bot module - this will include math, time, numpy (as np) and vrep python modules
from roverbot_lib import *
from Navigation import *
import numpy as np

#import any other required python modules


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()

sceneParameters.obstacle0_StartingPosition = [-0.45, 10.5]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = [0.6,-11]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = [-0.8,-11.2]   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene

#sceneParameters.sample0_StartingPosition = [0.5, 0]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
# sceneParameters.sample0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.sample1_StartingPosition = [0.2,-0.7]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
#sceneParameters.sample2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene


# sceneParameters.rock0_StartingPosition = -1  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
#sceneParameters.rock0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
#sceneParameters.rock1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
#sceneParameters.rock2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'	# specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1		# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
robotParameters.cameraHeightFromFloor = 0.15 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.0 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxBallDetectionDistance = 1 # the maximum distance away that you can detect the ball in metres
robotParameters.maxLanderDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres

# MAIN SCRIPT
def main():

	# Wrap everything in a try except case that catches KeyboardInterrupts.
	# In the exception catch code attempt to Stop the VREP Simulator so don't have to Stop it manually when pressing CTRL+C
	try:

		# Create VREP SoccerBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
		lunarBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
		lunarBotSim.StartSimulator()
		nav = Navigation(60)

		#We recommended changing this to a controlled rate loop (fixed frequency) to get more reliable control behaviour
		while True:
			# move the robot at a forward velocity of 0.2m/s with a rotational velocity of 0.3 rad/s.

			# Get Detected Objects
			samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()

			vel, rVel = nav.Navigate(samplesRB, obstaclesRB)
			#Check to see if the sample is within the camera's FOV
			lunarBotSim.UpdateObjectPositions()
			if samplesRB != None:
				# loop through each sample detected using Pythonian way
				for sample in samplesRB:
					sampleRange = sample[0]
					sampleBearing = sample[1]
					print(f"sample distance: {sampleRange}")
					if sampleRange < 0.21:
						lunarBotSim.SetTargetVelocities(0, 0)
						print("I am stopped")
						lunarBotSim.StopSimulator()
						return

			print(f"robot inputs {vel} and {rVel}\n")
			lunarBotSim.SetTargetVelocities(vel, rVel)
			lunarBotSim.UpdateObjectPositions()
			'''
			# Check to see if any obstacles are within the camera's FOV
			if obstaclesRB != None:
				# loop through each obstacle detected using Pythonian way
				for obstacle in obstaclesRB:
					obstacleRange = obstacle[0]
					obstacleBearing = obstacle[1]
				print(f"i can see {len(obstaclesRB)} obstacles\n")
			'''


	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP
		lunarBotSim.StopSimulator()

if __name__ == '__main__':
	main()