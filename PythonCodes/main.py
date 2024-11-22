from machine import Pin, Timer, PWM, ADC
from time import sleep
import uasyncio
import math
from RobotClasses import StepperMotor, MultiStepper, DifferentialDriver, JoystickController

async def monitorStart():
    global shouldMonitor
    with open("without_shield_with_LED2.csv", "w") as file: # w is for write, which overwrites the file if it already exists
        file.write("Time (s), Voltage (V), Resistance (Ohm)\n")
        current_time = 0

        while shouldMonitor:
            # Read raw ADC value (12-bit range: 0 to 4095)
            adc_value = adc_pin.read_u16()  # This returns 16-bit value (0-65535)
            
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


# KillSwitch function
def interruption_handler(pin):
    print("KillSwitch activated")
    global KillSwitch
    KillSwitch = True


# function to start the program
async def start():
    print("Starts")
    global shouldMonitor # Because then it can be used in the monitorStart function
    shouldMonitor = False
    uasyncio.create_task(monitorStart())
    # await car.inPlaceRotation(180)
    shouldMonitor = False # Then the program stops monitoring.

    global KillSwitch
    while not KillSwitch:
        pass
    


if __name__ == '__main__':

    # Set up the ADC pin (choose one of GP26, GP27, or GP28 for ADC on Pico W)
    adc_pin = ADC(Pin(26))  # GP26 is labeled as ADC0 on Pico found in the Kicad drawing

    # Reference voltage for the Pico W is typically 3.3V
    REFERENCE_VOLTAGE = 3.3
    R2 = 2200  # Known resistor value in the voltage divider circuit. We chose this resistor to get a linear graph for the LDR, so that we could get as big of a span as possible.



    print("Starts")

    # Makes objects for the motor
    motorRight = StepperMotor([0,1,2,3], 0.15, 18000, StepperMotor.half_step)
    motorLeft = StepperMotor([4,5,6,7], 0.15, 18000, StepperMotor.half_step)

    # makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])

    # set their delays
    multiStepper.set_Delays([0.01,0.01])

    # makes a differentialDriver object
    car = DifferentialDriver(multiStepper)

    joystickcontroller = JoystickController()

    # First KillSwitch is set to False, but if the button connected to pin 22 is pressed it is set to True and interrupts the program.
    KillSwitch = False
    KillSwitchButton = Pin(22, Pin.IN, Pin.PULL_UP)
    KillSwitchButton.irq(trigger=Pin.IRQ_FALLING, handler=interruption_handler)
    
    sleep(1)
    try:
        uasyncio.run(start())
    except KeyboardInterrupt:
        sleep(1)
        multiStepper.stop()
        print("Program stopped by user")