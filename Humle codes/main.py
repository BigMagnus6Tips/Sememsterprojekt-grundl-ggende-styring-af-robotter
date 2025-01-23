from machine import Pin, Timer, PWM, ADC, I2C
from time import sleep
import uasyncio as asyncio
from RobotClasses import StepperMotor, MultiStepper, DifferentialDriver, MonitorClass, DeadReckoningHandler, ServoMove, Crane, Switcher
from ssd1306_OLED import SSD1306_I2C


knownLinePositions = []

async def start():
    while True:
        pather.setPosition([0,0])
        pather.setAngle(0)
        
        #await car.goForward(-568)
        #pather.setAngle(90)
        #await pather.moveToPoint([0,29])
        #await car.inPlaceRotation(180)
        #asyncio.create_task(switcher.updateOled())
        
        #electro = PWM(Pin(12, Pin.OUT))
        #electro.freq(1000)
        #electro.duty_u16(0)
        
        #coordIndex = await switcher.waitForStart()
        #if coordIndex == len(coordsOfBolts)+1:
        #    break
        #await asyncio.sleep(1)
        #coordIndex = 7
        #await car.inPlaceRotation(-180)
        #await pather.home(leftMonitor, rightMonitor,[0,0], 90, 243)
        #await asyncio.sleep(1)
        #await pather.pickupAtPoint(coordsOfBolts[coordIndex],crane, electro)
        #switcher.choice += 1
        await crane.PimpMyRide()
        
        #await pather.moveToPoint([-54,9])
        #print("point 1")
        #print(str(pather.getPositionInCentimeter()) + " " + str(pather.getAngle()))
        #sleep(1)
        #await pather.moveToPoint([0,0])
        #print("point 2")
        #sleep(1)
        #await pather.moveToPoint([0,-250])
        #print("point 3")
        #sleep(1)
        #await pather.moveToPoint([30,30])
        #sleep(1)
        #await pather.moveToPoint([0,99])
        
        #multiStepper.stop()
        #while True:
        #    print("left: " + str(leftMonitor.monitorDigital(leftAdcCutoff)) + " right: " + str(rightMonitor.monitorDigital(rightAdcCutoff)))
        #    
        #    await asyncio.sleep(0.1)
        #pather.setAngle(0)
        


if __name__ == '__main__':

    # Makes the onboard LED object
    onBoardLed = Pin("LED", Pin.OUT)

    alive = False

    coordsOfBolts = [[153.2,110.2],
                     [133.5,69.4],
                     [-55,9],
                     [-135,-67],
                     [-9,76],
                     [9,106],
                     [-10,143],
                     [11,175],
                     [-5,209],
                     [7,240],
                     [148,48]]
    
    pitCoords = [0,-250]

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
    multiStepper.set_Delays([0.01,0.01])


    # Makes a differentialDriver object
    car = DifferentialDriver(multiStepper)

    # Makes a deadReckoningHandler object
    pather = DeadReckoningHandler(car, [0,0], 0, 33.5)

    startAngles = [75, 30, 0, 0]
    servo1 = ServoMove(8, [0, 180], startAngles[0])
    servo2 = ServoMove(9, [0, 180], startAngles[1])
    servo3 = ServoMove(10, [0, 180], startAngles[2])
    servo4 = ServoMove(11, [0,180], startAngles[3])

    magnetpin = 12

    electro = PWM(Pin(magnetpin, Pin.OUT))
    electro.freq(1000)
    electro.duty_u16(0)
    
    crane = Crane(servo1, servo2, servo3, servo4, startAngles)

    pinUp = 13
    pinDown = 18
    pinStart = 19

    switcher = Switcher(pinUp, pinDown, pinStart,len(coordsOfBolts)+1)

    oledI2C = I2C(0, scl=Pin(17), sda=Pin(16))
    #oled = SSD1306_I2C(128, 64, oledI2C)

    #switcher.addOled(oled)

    try:
        asyncio.run(start())
    except KeyboardInterrupt:
        sleep(1)
        multiStepper.stop()
        print("Program stopped by user")
print("Program done")