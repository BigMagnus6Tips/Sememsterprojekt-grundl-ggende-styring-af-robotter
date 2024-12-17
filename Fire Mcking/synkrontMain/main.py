from machine import Pin, Timer, PWM, ADC
from time import sleep
import uasyncio
import math
from OptimizedStepperClasses import StepperMotor, MultiStepper

# Reference voltage for the Pico W
REFERENCE_VOLTAGE = 3.3
R2 = 2200  # Known resistor value in the voltage divider circuit. We chose this resistor to get a linear graph between resistance and voltage, so that we could get as big of a span as possible.
THRESHOLD = 0.75  # Threshold value for the LDR to detect black or white. This value is found by testing the LDR on black and white surfaces and finding the average value between the two. This value is used to determine if the LDR is on black or white surface. The value is between 0 and 1, where 1 is the maximum value the LDR can read, which is the reference voltage.
 
def turnBlackOn(pin):
    global onBlack
    #print("On black")
    onBlack = True
    
def turnBlackOff(pin):
    global onBlack
    #print("On white")
    onBlack = False

def turnBlack2On(pin):
    global onBlack2
    #print("On black2")
    onBlack2 = True
    
def turnBlack2Off(pin):
    global onBlack2
    #print("On white2")
    onBlack2 = False

def monitorLDRsync():
    global onBlack
    global onBlack2
    if greenLEDInput.value() == 1:
        onBlack2 = True
    else:
        onBlack2 = False
    
    if redLEDInput.value() == 1:
        onBlack = True
    else:
        onBlack = False

def main():
    global onBlack
    global onBlack2
    
    onBlack2 = False
    onBlack = False
    maxSpeed = 600 # 600 er limit for koden ca. ved 30 % PWM
    resolution = 50
    timeToMaxSpeed = 0.5
    timeForEachResolution = timeToMaxSpeed/resolution
    speedSteps = maxSpeed/resolution
    
    sleep(1)
    
    
    for i in range(1, resolution+1):
        multiStepper.setSyncDelay(1/(i*speedSteps))
        print(i*speedSteps)
        multiStepper.moveSync([i*speedSteps*timeForEachResolution, i*speedSteps*timeForEachResolution])
    
    resetPin.value(1)
    resetPin.value(0)
    
    multiStepper.setSyncDelay(1/maxSpeed)
    
    leftMotorAmountHard = 0
    rightMotorAmountHard = 2
    
    leftMotorAmountSoft = 0
    rightMotorAmountSoft = 2
    counter = 0
    while True:
        monitorLDRsync()
        if onBlack2:
            while not onBlack:
                monitorLDRsync()
                multiStepper.moveSync([leftMotorAmountHard,rightMotorAmountHard])
            resetPin.value(1)
            resetPin.value(0)

        elif onBlack:

            multiStepper.moveSync([leftMotorAmountSoft,rightMotorAmountSoft])
            
        else:
            if counter%2 == 0:
                multiStepper.moveSync([6,5])
            multiStepper.moveSync([2,2])
            counter += 1
            #print("turning")
    


if __name__ == '__main__':

    # Makes objects for the motor
    motorRight = StepperMotor(reversed([0,1,2,3]), 0.45, 18000)
    motorLeft = StepperMotor([4,5,6,7], 0.45, 18000)


    # Makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])
    
    redLEDInput = Pin(28, Pin.IN, Pin.PULL_DOWN)
    greenLEDInput = Pin(19, Pin.IN, Pin.PULL_DOWN)
    resetPin = Pin(18, Pin.OUT)

    try:
        main()
    except KeyboardInterrupt:
        multiStepper.stop()
        print("Stopped")