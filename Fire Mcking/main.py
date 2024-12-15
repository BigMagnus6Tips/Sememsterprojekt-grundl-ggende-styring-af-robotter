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



async def monitorLDR():
    global onBlack
    global onBlack2
    while True:
        if ldrPin3.value() == 1:
            onBlack2 = True
        else:
            onBlack2 = False
        
        if ldrPin1.value() == 1:
            onBlack = True
        else:
            onBlack = False
        await uasyncio.sleep(0.001)

def monitorLDRsync():
    global onBlack
    global onBlack2
    if ldrPin3.value() == 1:
        onBlack2 = True
    else:
        onBlack2 = False
    
    if ldrPin1.value() == 1:
        onBlack = True
    else:
        onBlack = False


# Function to blink the onboard LED
async def blinkLed():
    while True:
        onBoardLed.toggle()
        await uasyncio.sleep(0.5)



# function to start the program
async def start():
    global onBlack
    global onBlack2
    
    onBlack2 = False
    onBlack = False
    #uasyncio.create_task(blinkLed())
    #uasyncio.create_task(monitorLDR())
    maxSpeed = 800 # 600 er limit for koden ca. ved 30 % PWM
    resolution = 50
    timeToMaxSpeed = 0.5
    timeForEachResolution = timeToMaxSpeed/resolution
    speedSteps = maxSpeed/resolution
    
    sleep(1)
    
    
    for i in range(1, resolution+1):
        multiStepper.set_Speed([i*speedSteps, i*speedSteps])
        print(i*speedSteps)
        await multiStepper.move([i*speedSteps*timeForEachResolution,i*speedSteps*timeForEachResolution])
    
    
    multiStepper.set_Speed([maxSpeed, maxSpeed])

    while True:
        monitorLDRsync()
        if onBlack2:
            leftMotorAmount = 4
            rightMotorAmount = 20
            multiStepper.set_Speed([maxSpeed*leftMotorAmount/rightMotorAmount, maxSpeed])
            await multiStepper.move([leftMotorAmount,rightMotorAmount])
            
        elif onBlack:
            leftMotorAmount = 3
            rightMotorAmount = 7
            multiStepper.set_Speed([maxSpeed*leftMotorAmount/rightMotorAmount, maxSpeed])
            await multiStepper.move([leftMotorAmount,rightMotorAmount])
            
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

    ldrPin1 = Pin(28, Pin.IN, Pin.PULL_DOWN)
    ldrPin2 = Pin(27, Pin.IN, Pin.PULL_DOWN)
    ldrPin3 = Pin(26, Pin.IN, Pin.PULL_DOWN)
    ldrPin4 = Pin(19, Pin.IN, Pin.PULL_DOWN)
    
    #ldrPin1.irq(trigger=Pin.IRQ_FALLING, handler=turnBlackOff)
    #ldrPin2.irq(trigger=Pin.IRQ_RISING, handler=turnBlackOn)

    #ldrPin3.irq(trigger=Pin.IRQ_FALLING, handler=turnBlack2Off)
    #ldrPin4.irq(trigger=Pin.IRQ_RISING, handler=turnBlack2On)

    # Makes a differentialDriver object
    car = DifferentialDriver(multiStepper)




    try:
        uasyncio.run(start())
    except KeyboardInterrupt:
        sleep(1)
        multiStepper.stop()