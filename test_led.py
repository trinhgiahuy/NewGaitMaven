import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)

while True:
	print "Pin 18 on"
	GPIO.output(18,1)
	time.sleep(1)
	GPIO.output(18,0)
	time.sleep(1)

	#print "Pin 20 on"
	#GPIO.output(22,1)
	#time.sleep(1)
	#GPIO.output(22,0)
	#time.sleep(1)
