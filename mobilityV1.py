import RPi.GPIO as GPIO
import time

WHEEL_BASE = 0.3

GPIO.setmode(GPIO.BCM)

def mob(desired_vel, desired_rot_vel): 

	if desired_vel == 10:
		GPIO.setwarnings(False)
		GPIO.setup(12, GPIO.OUT) 
		GPIO.setup(13, GPIO.OUT) 

		GPIO.setup(5, GPIO.OUT)
		GPIO.output(5, GPIO.HIGH)

		GPIO.setup(6, GPIO.OUT)
		GPIO.output(6, GPIO.LOW)
		
		ena = GPIO.PWM(12, 100) 	 
		enb = GPIO.PWM(13, 100)
			
		ena.start(20)
		enb.start(20)

	else:
		GPIO.setwarnings(False)
		GPIO.setup(12, GPIO.OUT) 
		GPIO.setup(13, GPIO.OUT) 

		GPIO.setup(5, GPIO.OUT)
		GPIO.output(5, GPIO.HIGH)

		GPIO.setup(6, GPIO.OUT)
		GPIO.output(6, GPIO.HIGH)
								 
		ena = GPIO.PWM(12, 100) 	 
		enb = GPIO.PWM(13, 100)

		velR = desired_vel + desired_rot_vel * WHEEL_BASE/2
		#print('velR')
		velL = desired_vel - desired_rot_vel * WHEEL_BASE/2
		#print('velL')

		ena.start(min(velR*100, 100))
		enb.start(min(velL*100, 100))
	time.sleep(0.5)
	

#if __name__ == "__main__":
	#while 1:
		#mob(0.3, 0.3)

#GPIO.cleanup()
