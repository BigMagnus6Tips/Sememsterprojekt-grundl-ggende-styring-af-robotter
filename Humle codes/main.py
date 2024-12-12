from machine import Pin, Timer, PWM, ADC
from time import sleep
import uasyncio as asyncio
from RobotClasses import StepperMotor, MultiStepper, DifferentialDriver, MonitorClass, deadReckoningHandler




async def start():
    while True:
        print("left: " + str(await leftMonitor.monitor()) + " right: " + str(await rightMonitor.monitor()))
        
        await asyncio.sleep(0.1)


if __name__ == '__main__':

    # Makes the onboard LED object
    onBoardLed = Pin("LED", Pin.OUT)

    alive = False



    # Adc pin for moonitoring the LDR
    leftAdcPin = 26  # GP26 is labeled as ADC0 on Pico found in the Kicad drawing
    rightAdcPin = 27  # GP27 is labeled as ADC1 on Pico found in the Kicad drawing

    # Reference voltage for the Pico W
    REFERENCE_VOLTAGE = 3.3
    R2 = 2200  # Known resistor value in the voltage divider circuit. We chose this resistor to get a linear graph between resistance and voltage, so that we could get as big of a span as possible.

    leftMonitor = MonitorClass(leftAdcPin, REFERENCE_VOLTAGE,R2 )
    rightMonitor = MonitorClass(rightAdcPin, REFERENCE_VOLTAGE, R2)

    print("Starts")

    # Makes objects for the motor
    motorRight = StepperMotor([0,1,2,3], 0.2, 18000, StepperMotor.half_step)
    motorLeft = StepperMotor([4,5,6,7], 0.2, 18000, StepperMotor.half_step)


    # Makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])

    # Set their delays
    multiStepper.set_Delays([0.01,0.01])


    # Makes a differentialDriver object
    car = DifferentialDriver(multiStepper)


    try:
        asyncio.run(start())
    except KeyboardInterrupt:
        sleep(1)
        multiStepper.stop()
        print("Program stopped by user")
print("Program done")