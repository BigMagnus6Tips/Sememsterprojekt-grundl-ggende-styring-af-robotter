from machine import Pin, Timer, PWM, ADC
from time import sleep
import uasyncio
import math
from RobotClasses import StepperMotor, MultiStepper, DifferentialDriver, JoystickController
import bluetooth
from WifiClasses import ConstantsForCommunication as comms
import aioble


async def monitorStart():
    global shouldMonitor
    global killSwitch
    with open("without_shield_with_LED2.csv", "w") as file: # w is for write, which overwrites the file if it already exists
        file.write("Time (s), Voltage (V), Resistance (Ohm)\n")
        current_time = 0

        while shouldMonitor and not killSwitch:
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


async def find_remote():
    # Scan for 5 seconds, in active mode, with very low interval/window (to
    # maximise detection rate).
    async with aioble.scan(5000, interval_us=30000, window_us=30000, active=True) as scanner: # type: ignore
        async for result in scanner:

            # See if it matches our name
            if result.name() == "Humles Joystick":
                print("Found HumleJoystick")
                for item in result.services():
                    print (item)
                if _ENV_SENSE_UUID in result.services():
                    print("Found Robot Remote Service")
                    return result.device

            
    return None


async def peripheral_task():
    
    print('starting peripheral task')
    global dataFromController
    global connected
    connected = False
    device = await find_remote()
    if not device:
        print("Robot Remote not found")
        return
    try:
        print("Connecting to", device)
        connection = await device.connect()
        
    except uasyncio.TimeoutError:
        print("Timeout during connection")
        return

    async with connection:
        print("Connected")
        connected = True
        alive = True
        global killSwitch
        while not killSwitch and alive:
            try:
                robot_service = await connection.service(_REMOTE_UUID)
                print(robot_service)
                control_characteristic = await robot_service.characteristic(_REMOTE_CHARACTERISTICS_UUID)
                print(control_characteristic)
            except uasyncio.TimeoutError:
                print("Timeout discovering services/characteristics")
                return
            while not killSwitch:
                if control_characteristic != None:
                    try:
                        data = await control_characteristic.read()
                        listData = list(data)
                        if len(listData) != 0:
                            dataFromController = [[listData[comms.indexSpeedLeftBig]*256+listData[comms.indexSpeedLeftLittle], 
                                                listData[comms.indexSpeedRightBig]*256+listData[comms.indexSpeedRightLittle]], 
                                                [listData[comms.indexMotorLeftDirection]-1, listData[comms.indexMotorRightDirection]-1],
                                                listData[comms.indexButton]]
                            if dataFromController[2] == 0:
                                print("Button pressed")
                                connected = False
                                alive = False
                                killSwitch = True
                                return
                        
                    except TypeError:
                        print(f'something went wrong; remote disconnected?')
                        connected = False
                        alive = False
                        return
                    except uasyncio.TimeoutError:
                        print(f'something went wrong; timeout error?')
                        connected = False
                        alive = False
                        return
                    except AttributeError:
                        print(f'something went wrong; Gatt error - did the remote die?')
                        connected = False
                        alive = False
                        return
                else:
                    print('no characteristic')
                await uasyncio.sleep_ms(10)

async def moveFromControllerData():
    print( "MoveFromControllerData")
    global killSwitch
    while not killSwitch:
        if 0 not in dataFromController[0]:
            multiStepper.set_Speed(dataFromController[0])
            await multiStepper.move(dataFromController[1])
        else:
            await uasyncio.sleep(0.05)



    


# killSwitch function
def interruption_handler(pin):
    global killSwitch
    killSwitch = True

async def blinkLed():
    global killSwitch
    while not killSwitch:
        onBoardLed.toggle()
        await uasyncio.sleep(0.5)



# function to start the program
async def start():
    print("Starts2")
    global shouldMonitor # Because then it can be used in the monitorStart function
    shouldMonitor = False
    #uasyncio.create_task(monitorStart())
    uasyncio.create_task(peripheral_task())
    uasyncio.create_task(moveFromControllerData())
    print("LDNF")
    #uasyncio.create_task(blinkLed())
    # await car.inPlaceRotation(180)
    shouldMonitor = False # Then the program stops monitoring.

    global killSwitch
    while not killSwitch:
        await uasyncio.sleep(0.1)
    


if __name__ == '__main__':


    onBoardLed = Pin("LED", Pin.OUT)

    _REMOTE_UUID = bluetooth.UUID(0x1848)
    _ENV_SENSE_UUID = bluetooth.UUID(0x1878) 
    _REMOTE_CHARACTERISTICS_UUID = bluetooth.UUID(0x2A78)

    connected = False
    alive = False




    # Set up the ADC pin (choose one of GP26, GP27, or GP28 for ADC on Pico W)
    adc_pin = ADC(Pin(26))  # GP26 is labeled as ADC0 on Pico found in the Kicad drawing

    # Reference voltage for the Pico W is typically 3.3V
    REFERENCE_VOLTAGE = 3.3
    R2 = 2200  # Known resistor value in the voltage divider circuit. We chose this resistor to get a linear graph for the LDR, so that we could get as big of a span as possible.



    print("Starts")

    # Makes objects for the motor
    motorRight = StepperMotor([0,1,2,3], 0.2, 18000, StepperMotor.half_step)
    motorLeft = StepperMotor([4,5,6,7], 0.2, 18000, StepperMotor.half_step)


    # makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])

    # set their delays
    multiStepper.set_Delays([0.01,0.01])


    # makes a differentialDriver object
    car = DifferentialDriver(multiStepper)

    dataFromController = [[0,0], [0,0], 1]


    # First killSwitch is set to False, but if the button connected to pin 22 is pressed it is set to True and interrupts the program.
    killSwitch = False
    killSwitchButton = Pin(22, Pin.IN, Pin.PULL_UP)
    #killSwitchButton.irq(trigger=Pin.IRQ_FALLING, handler=interruption_handler)
    
    print("WTF")
    print("IDK")
    try:
        print("SHOUDL START")
        uasyncio.run(start())
    except KeyboardInterrupt:
        sleep(1)
        multiStepper.stop()
        print("Program stopped by user")
print("Program done")