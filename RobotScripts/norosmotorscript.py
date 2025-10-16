#the idea here is to directly send power to gpio pins on the rasperry pi.
#the exact pins will depend on the motor driver board and wiring used
##this is not tested as I only had access to paralax controller with proprietary software 
##but this is the general idea

#assumes l293d motor driver board and rasperry pi

#code derived from and motor driver board
#configured using this example for first motor:
## https://www.electronicshub.org/controlling-a-dc-motor-with-raspberry-pi/
#^easiest way to understand expected wiring of motor driver board


import RPi.GPIO as GPIO
from time import sleep

# Pins for Motor Driver Inputs 
#this is important because the logical signals from gpio
# need to be translated properly, if anything is flipped on controller it has to
# be flipped here
Motor1A = 24#pin 2 on motor driver
Motor1B = 23#pin 7 on motor driver
Motor1E = 25##pin1 on motor driver/physical pin 22
## these are pins that i guessed would be safe but are untested
Motor2A=17#pin 10
Motor2B=27#pin 15
Motor2E=22#pin 9 on motor driver
# motor driver pins do translate to specific electrical signals when powered in specific ports
# # we need to understand our chip and power it accordingly using simple gpio output instructions
def setup():
	GPIO.setmode(GPIO.BCM)				# GPIO Numbering
	GPIO.setup(Motor1A,GPIO.OUT)  # All pins as Outputs
	GPIO.setup(Motor1B,GPIO.OUT)
	GPIO.setup(Motor1E,GPIO.OUT)
	GPIO.setup(Motor2A,GPIO.OUT)
	GPIO.setup(Motor2B,GPIO.OUT)
	GPIO.setup(Motor2E,GPIO.OUT)

def loop():
	# Going forwards
	GPIO.output(Motor1A,GPIO.HIGH)
	GPIO.output(Motor1B,GPIO.LOW)
	GPIO.output(Motor1E,GPIO.HIGH)
	GPIO.output(Motor2A,GPIO.HIGH)
	GPIO.output(Motor2B,GPIO.LOW)
	GPIO.output(Motor2E,GPIO.HIGH)
	#turning to side 
	sleep(5)
	GPIO.output(Motor1A,GPIO.HIGH)
	GPIO.output(Motor1B,GPIO.LOW)
	GPIO.output(Motor1E,GPIO.HIGH)
	GPIO.output(Motor2A,GPIO.LOW)
	GPIO.output(Motor2B,GPIO.LOW)
	GPIO.output(Motor2E,GPIO.HIGH)
	sleep(5)
	#turning to other side
	GPIO.output(Motor1A,GPIO.LOW)
	GPIO.output(Motor1B,GPIO.LOW)
	GPIO.output(Motor1E,GPIO.HIGH)
	GPIO.output(Motor2A,GPIO.HIGH)
	GPIO.output(Motor2B,GPIO.LOW)
	GPIO.output(Motor2E,GPIO.HIGH)
	
 
	sleep(5)
 	# Going backwards
	GPIO.output(Motor1A,GPIO.LOW)
	GPIO.output(Motor1B,GPIO.HIGH)
	GPIO.output(Motor1E,GPIO.HIGH)
	GPIO.output(Motor2A,GPIO.LOW)
	GPIO.output(Motor2B,GPIO.HIGH)
	GPIO.output(Motor2E,GPIO.HIGH)

	sleep(5)
	# Stop
	GPIO.output(Motor1E,GPIO.LOW)
	GPIO.output(Motor2E,GPIO.LOW)

def destroy():	
	GPIO.cleanup()

if __name__ == '__main__':     # Program start from here
	setup()
	try:
		loop()
	except KeyboardInterrupt:
		destroy()
 