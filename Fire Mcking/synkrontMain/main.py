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
    global redLED
    #print("On black")
    redLED = True
    
def turnBlackOff(pin):
    global redLED
    #print("On white")
    redLED = False

def turnBlack2On(pin):
    global greenLED
    #print("On black2")
    greenLED = True
    
def turnBlack2Off(pin):
    global greenLED
    #print("On white2")
    greenLED = False

def monitorLDRsync():
    global redLED
    global greenLED
    if greenLEDInput.value() == 1:
        greenLED = True
    else:
        greenLED = False
    
    if redLEDInput.value() == 1:
        redLED = True
    else:
        redLED = False
def rampUp(maxSpeed, resolution, timeToMaxSpeed):
    timeForEachResolution = timeToMaxSpeed/resolution
    speedSteps = maxSpeed/resolution
    
    for i in range(1, resolution+1):
        multiStepper.setSyncDelay(1/(i*speedSteps))
        multiStepper.moveSync([i*speedSteps*timeForEachResolution, i*speedSteps*timeForEachResolution])

def main():
    global redLED
    global greenLED
    
    greenLED = False
    redLED = False
    maxSpeed = 600 # 600 er limit for koden ca. ved 30 % PWM
    hardTurnSpeed = 300
    resolution = 50
    timeToMaxSpeed = 0.5
    
    sleep(1)
    
    rampUp(maxSpeed, resolution, timeToMaxSpeed)

    redResetPin.value(1)
    redResetPin.value(0)
    greenResetPin.value(1)
    greenResetPin.value(0)
    
    multiStepper.setSyncDelay(1/maxSpeed)
    
    leftMotorAmountHard = 0
    rightMotorAmountHard = 3
    
    leftMotorAmountSoft = 0
    rightMotorAmountSoft = 4
    counter = 0
    while True:
        monitorLDRsync()
        if greenLED:
            multiStepper.setSyncDelay(1/hardTurnSpeed)
            while not redLED:
                monitorLDRsync()
                multiStepper.moveSync([leftMotorAmountHard,rightMotorAmountHard])
            rampUp(maxSpeed, 5, 0.1)
            greenResetPin.value(1)
            greenResetPin.value(0)
            monitorLDRsync()
            while not redLED:
                monitorLDRsync()
                multiStepper.moveSync([1,0])
            multiStepper.setSyncDelay(1/maxSpeed)
        elif redLED:
            multiStepper.moveSync([leftMotorAmountSoft,rightMotorAmountSoft])
            redResetPin.value(1)
            redResetPin.value(0)
            
        else:
            if counter%3 == 0:
                multiStepper.moveSync([2,0])
            multiStepper.moveSync([3,3])
            counter += 1
            #print("turning")
    


if __name__ == '__main__':

    # Makes objects for the motor
    motorRight = StepperMotor(reversed([0,1,2,3]), 0.30, 18000)
    motorLeft = StepperMotor([4,5,6,7], 0.30, 18000)


    # Makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])
    
    redLEDInput = Pin(28, Pin.IN, Pin.PULL_DOWN)
    greenLEDInput = Pin(19, Pin.IN, Pin.PULL_DOWN)
    greenResetPin = Pin(18, Pin.OUT)
    redResetPin = Pin(17, Pin.OUT)

    onBoardLED = Pin(25, Pin.OUT)

    try:
        main()
    except KeyboardInterrupt:
        multiStepper.stop()
        print("Stopped")