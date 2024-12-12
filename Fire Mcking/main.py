from machine import Pin, Timer, PWM, ADC
from time import sleep
import uasyncio
import math
from RobotClasses import StepperMotor, MultiStepper, DifferentialDriver

# Reference voltage for the Pico W
REFERENCE_VOLTAGE = 3.3
R2 = 2200  # Known resistor value in the voltage divider circuit. We chose this resistor to get a linear graph between resistance and voltage, so that we could get as big of a span as possible.




async def monitorStart():
    global shouldMonitor
    with open("without_shield_with_LED2.csv", "w") as file: # w is for write, which overwrites the file if it already exists
        file.write("Time (s), Voltage (V), Resistance (Ohm)\n")
        current_time = 0

        while shouldMonitor:
            # Read raw ADC value (12-bit range: 0 to 4095)
            adc_value = LDR_adc.read_u16()  # This returns 16-bit value (0-65535)
            
            # Scale the 16-bit reading to voltage
            voltage = (adc_value / 65535) * REFERENCE_VOLTAGE

            # Calculate the resistance of the LDR with the formula from the voltage divider circuit
            resistance = round(REFERENCE_VOLTAGE * R2 / voltage - R2)
            
            # Print the data to the file, where 2f means there are two decimal points
            file.write("{:.2f}, {:.2f}, {}\n".format(current_time, voltage, resistance))

            # Delay between readings, which enables the robot to drive in the mean time.
            await uasyncio.sleep(0.1)
            # This is for plotting the time in the csv file.
            current_time += 0.1




# Function to blink the onboard LED
async def blinkLed():
    while True:
        onBoardLed.toggle()
        await uasyncio.sleep(0.5)



# function to start the program
async def start():
    global shouldMonitor
    shouldMonitor = False
    # uasyncio.create_task(monitorStart())
    uasyncio.create_task(blinkLed())
    maxSpeed = 800 # 600 er limit for koden ca. ved 20 % PWM
    resolution = 10
    timeToMaxSpeed = 1
    timeForEachResolution = timeToMaxSpeed/resolution
    speedSteps = maxSpeed/resolution
    sleep(2)
    for i in range(1, resolution+1):
        multiStepper.set_Speed([i*speedSteps, i*speedSteps])
        print(i*speedSteps)
        await multiStepper.move([i*speedSteps*timeForEachResolution,i*speedSteps*timeForEachResolution])
    print("Done")
    while True:
        await multiStepper.move([1000,1000])



    while True:
        await uasyncio.sleep(0.1)
    


if __name__ == '__main__':

    # Makes the onboard LED object
    onBoardLed = Pin("LED", Pin.OUT)

    # Adc pin for monitoring the LDR
    LDR_adc = ADC(Pin(26))  # GP26 is labeled as ADC0 on Pico found in the Kicad drawing

    # Makes objects for the motor
    motorRight = StepperMotor([0,1,2,3], 0.4, 18000, StepperMotor.full_step)
    motorLeft = StepperMotor([4,5,6,7], 0.4, 18000, StepperMotor.full_step)


    # Makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])




    # Makes a differentialDriver object
    car = DifferentialDriver(multiStepper)




    try:
        uasyncio.run(start())
    except KeyboardInterrupt:
        sleep(1)
        multiStepper.stop()