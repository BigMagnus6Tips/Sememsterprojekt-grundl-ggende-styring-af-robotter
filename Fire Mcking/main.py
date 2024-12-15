from machine import Pin, Timer, PWM, ADC
from time import sleep
import uasyncio
import math
from RobotClasses import StepperMotor, MultiStepper, DifferentialDriver

# Reference voltage for the Pico W
REFERENCE_VOLTAGE = 3.3
R2 = 2200  # Known resistor value in the voltage divider circuit. We chose this resistor to get a linear graph between resistance and voltage, so that we could get as big of a span as possible.
THRESHOLD = 0.75  # Threshold value for the LDR to detect black or white. This value is found by testing the LDR on black and white surfaces and finding the average value between the two. This value is used to determine if the LDR is on black or white surface. The value is between 0 and 1, where 1 is the maximum value the LDR can read, which is the reference voltage.
 
def turnBlackOn(pin):
    global onBlack
    print("On black")
    onBlack = True
    
def turnBlackOff(pin):
    global onBlack
    print("On white")
    onBlack = False

async def monitorLDR():
    global onBlack
    while True:
        if ldrPin1.value() == 0:
            onBlack = True
        else:
            onBlack = False
        await uasyncio.sleep(0.001)


# Function to blink the onboard LED
async def blinkLed():
    while True:
        onBoardLed.toggle()
        await uasyncio.sleep(0.5)



# function to start the program
async def start():
    global onBlack
    onBlack = False
    #uasyncio.create_task(blinkLed())
    #uasyncio.create_task(monitorLDR())
    maxSpeed = 600 # 600 er limit for koden ca. ved 30 % PWM
    resolution = 10
    timeToMaxSpeed = 1
    timeForEachResolution = timeToMaxSpeed/resolution
    speedSteps = maxSpeed/resolution
    multiStepper.set_Speed([maxSpeed, maxSpeed])
    leftMotorSpeed = 24
    rightMotorSpeed = 40
    sleep(2)
    while True:
        print(ldrPin1.value())
        if onBlack:
            multiStepper.set_Speed([maxSpeed*leftMotorSpeed/rightMotorSpeed, maxSpeed])
            #print("now moving")
            await multiStepper.move([leftMotorSpeed,rightMotorSpeed])
            
        else:
            multiStepper.set_Speed([maxSpeed, maxSpeed])
            await multiStepper.move([10,10])
            #print("turning")
    
    
    """    
    for i in range(1, resolution+1):
        multiStepper.set_Speed([i*speedSteps, i*speedSteps])
        print(i*speedSteps)
        await multiStepper.move([i*speedSteps*timeForEachResolution,i*speedSteps*timeForEachResolution])
    print("Done")
    while True:
    """        
    



    while True:
        await uasyncio.sleep(0.1)
    


if __name__ == '__main__':

    # Makes the onboard LED object
    onBoardLed = Pin("LED", Pin.OUT)

    # Makes objects for the motor
    motorRight = StepperMotor(reversed([0,1,2,3]), 0.3, 18000, StepperMotor.full_step)
    motorLeft = StepperMotor([4,5,6,7], 0.3, 18000, StepperMotor.full_step)


    # Makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])

    ldrPin1 = Pin(26, Pin.IN, Pin.PULL_DOWN)
    ldrPin2 = Pin(27, Pin.IN, Pin.PULL_DOWN)
    
    ldrPin1.irq(trigger=Pin.IRQ_FALLING, handler=turnBlackOn)
    ldrPin2.irq(trigger=Pin.IRQ_RISING, handler=turnBlackOff)


    # Makes a differentialDriver object
    car = DifferentialDriver(multiStepper)




    try:
        uasyncio.run(start())
    except KeyboardInterrupt:
        sleep(1)
        multiStepper.stop()