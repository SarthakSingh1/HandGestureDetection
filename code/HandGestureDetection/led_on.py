#import libraries
import RPi.GPIO as GPIO
import time

#GPIO Basic initialization
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Use a variable for the Pin to use
#If you followed my pictures, it's port 7 => BCM 4
bit_1 = 17
bit_2 = 27
bit_3 = 22


#Initialize your pin
GPIO.setup(bit_1,GPIO.OUT)
GPIO.setup(bit_2,GPIO.OUT)
GPIO.setup(bit_3,GPIO.OUT)

#Turn on the LED
print("LED on")
GPIO.output(bit_1,1)
GPIO.output(bit_2,1)
GPIO.output(bit_3,1)

#Wait 5s
time.sleep(5)

#Turn off the LED
print("LED off")
GPIO.output(bit_1,0)
GPIO.output(bit_2,0)
GPIO.output(bit_3,0)