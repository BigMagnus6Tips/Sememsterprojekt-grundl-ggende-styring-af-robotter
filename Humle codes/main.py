from machine import Pin, Timer, PWM, ADC
from time import sleep
import uasyncio as asyncio
from RobotClasses import StepperMotor, MultiStepper, DifferentialDriver, MonitorClass, DeadReckoningHandler


knownLinePositions = []

async def start():
    #await car.goForward(-568)
    #pather.setAngle(90)
    #await pather.moveToPoint([0,29])
    #await car.inPlaceRotation(180)
    #await pather.moveToPoint([0,99])
    #print("point 1")
    #print(str(pather.getPositionInCentimeter()) + " " + str(pather.getAngle()))
    #sleep(1)
    #await pather.moveToPoint([30,30+99])
    #print("point 2")
    #sleep(1)
    #await pather.moveToPoint([30,81+30+99])
    #print("point 3")
    #sleep(1)
    #await pather.moveToPoint([30,30])
    #sleep(1)
    #await pather.moveToPoint([0,99])
    #await car.inPlaceRotation(-1.5)
    #multiStepper.stop()
    #while True:
    #    print("left: " + str(leftMonitor.monitorDigital(leftAdcCutoff)) + " right: " + str(rightMonitor.monitorDigital(rightAdcCutoff)))
    #    
    #    await asyncio.sleep(0.1)
    #pather.setAngle(0)
    await pather.home(leftMonitor, rightMonitor)


if __name__ == '__main__':

    # Makes the onboard LED object
    onBoardLed = Pin("LED", Pin.OUT)

    alive = False



    # Adc pin for moonitoring the LDR
    leftAdcPin = 26  # GP26 is labeled as ADC0 on Pico found in the Kicad drawing
    rightAdcPin = 27  # GP27 is labeled as ADC1 on Pico found in the Kicad drawing

    leftAdcCutoff = 2.2
    rightAdcCutoff = 2.2

    # Reference voltage for the Pico W
    REFERENCE_VOLTAGE = 3.3
    R2 = 2200  # Known resistor value in the voltage divider circuit. We chose this resistor to get a linear graph between resistance and voltage, so that we could get as big of a span as possible.

    leftMonitor = MonitorClass(leftAdcPin, REFERENCE_VOLTAGE, R2, [21, -2])
    rightMonitor = MonitorClass(rightAdcPin, REFERENCE_VOLTAGE, R2, [21, 2])

    print("Starts")

    # Makes objects for the motor
    motorRight = StepperMotor(reversed([0,1,2,3]), 0.2, 18000, StepperMotor.half_step)
    motorLeft = StepperMotor(reversed([4,5,6,7]), 0.2, 18000, StepperMotor.half_step)


    # Makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])

    # Set their delays
    multiStepper.set_Delays([0.008,0.008])


    # Makes a differentialDriver object
    car = DifferentialDriver(multiStepper)

    # Makes a deadReckoningHandler object
    pather = DeadReckoningHandler(car)

    try:
        asyncio.run(start())
    except KeyboardInterrupt:
        sleep(1)
        multiStepper.stop()
        print("Program stopped by user")
print("Program done")