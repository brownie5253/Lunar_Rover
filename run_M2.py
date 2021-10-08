from vision import *
from mobilityV1 import *
from Navigate_m1 import *
import numpy as np
import time
import RPi.GPIO as GPIO 			# Import GPIO module



# MAIN SCRIPT
def main():

		 
		#GPIO.setmode(GPIO.BCM)				# Set the GPIO pin naming convention

		#GPIO.setup(23, GPIO.OUT) #red
		#PIO.setup(24, GPIO.OUT) #green
		#PIO.setup(25, GPIO.OUT) #blue
		
		nav = Navigation(62)

		while True:
			#start = time.perf_counter()
			# Get Detected Objects
			samplesRB = getReading()
			print("camera feedback: ",samplesRB[0],samplesRB[1])

			#print("after vision")
			#print(time.perf_counter()-start)

			#print(f"got reading from camera: {samplesRB}")

			vel, rVel = nav.Navigate(samplesRB[1])
			##print("after Nav")
			##print(time.perf_counter()-start)
			if samplesRB[1] != None:
				sampleRange = samplesRB[0]
				sampleBearing = samplesRB[1]
				#print(f"sample distance: {sampleRange}")
				dist = float(float(sampleRange)/1000)
				print("sample distance: ",dist)
				if dist < 0.1:
					print("I am stopped")
					#GPIO.output(23, GPIO.HIGH)
					#GPIO.output(24, GPIO.LOW)
					#GPIO.output(25, GPIO.LOW)
					#GPIO.cleanup()
					return
				else:
					print("Heading to the goal")
					#GPIO.output(23, GPIO.LOW)
					#GPIO.output(24, GPIO.HIGH)
					#GPIO.output(25, GPIO.LOW)
			else:
				print("I am searching")
				#GPIO.output(23, GPIO.LOW)
				#GPIO.output(24, GPIO.LOW)
				#GPIO.output(25, GPIO.HIGH)
				vel = 10
				rVel = 5
				
			#print("after Loop")
			#print(time.perf_counter()-start)
			print("vel: ",vel)
			print("rot vel: ",rVel)
			print("\n")
			#print(f"robot inputs {vel} and {rVel}\n")
			
			mob(vel, rVel)


if __name__ == '__main__':
	main()
