import sys
import aioble
import bluetooth
import machine
import uasyncio as asyncio
from micropython import const
from RobotClasses import JoystickController
from WifiClasses import ConstantsForCommunication as comms
import ssd1306_OLED as ssd1306_OLED


def uid():
    """ Return the unique id of the device as a string """
    return "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}".format(
        *machine.unique_id())


# Bluetooth UUIDS can be found online at https://www.bluetooth.com/specifications/gatt/services/
MANUFACTURER_ID = const(0x02A29)
MODEL_NUMBER_ID = const(0x2A24)
SERIAL_NUMBER_ID = const(0x2A25)
HARDWARE_REVISION_ID = const(0x2A26)
BLE_VERSION_ID = const(0x2A28)

# Makes a JoystickController object with the pins for the joystick
joystickController = JoystickController("GP27", "GP28")

# sets the format of the data to be sent to the robot
data = [0, 0, 0, 0, 0, 0, 0]

# Adds the button to the joystickController object
joystickController.addButton("GP22")

# Sets up the leds
leds = [machine.Pin("GP9", machine.Pin.OUT), machine.Pin("GP8", machine.Pin.OUT), machine.Pin("GP7", machine.Pin.OUT), machine.Pin("GP6", machine.Pin.OUT)]

# Sets up the OLED display
OLEDI2C = machine.I2C(0, scl=machine.Pin(13), sda=machine.Pin(12))
display = ssd1306_OLED.SSD1306_I2C(128, 64, OLEDI2C)
display.fill(0)
display.rotate(0)
display.text('Humles Joystick', 0, 0, 1)
display.show()


# Information for the bluetooth connection UUIDs
_DEVICE_INFO_UUID = bluetooth.UUID(0x180A) 
_GENERIC = bluetooth.UUID(0x1848)
_JOYSTICK_UUID = bluetooth.UUID(0x2A78)
_ROBOT = bluetooth.UUID(0x1878)
                              
_BLE_APPEARANCE_GENERIC_REMOTE_CONTROL = const(384)

# Advertisement interval in milliseconds
ADV_INTERVAL_MS = 250_000

# Create Service for device info
device_info = aioble.Service(_DEVICE_INFO_UUID)
                              
connection = None

# Create Characteristic for device info
aioble.Characteristic(device_info, bluetooth.UUID(MANUFACTURER_ID), read=True, initial="KevsRobots.com")
aioble.Characteristic(device_info, bluetooth.UUID(MODEL_NUMBER_ID), read=True, initial="1.0")
aioble.Characteristic(device_info, bluetooth.UUID(SERIAL_NUMBER_ID), read=True, initial=uid())
aioble.Characteristic(device_info, bluetooth.UUID(HARDWARE_REVISION_ID), read=True, initial=sys.version)
aioble.Characteristic(device_info, bluetooth.UUID(BLE_VERSION_ID), read=True, initial="1.0")

# Create Service for remote control
remote_service = aioble.Service(_GENERIC)

# Create Characteristic for remote control
joystick_characteristic = aioble.Characteristic(
    remote_service, _JOYSTICK_UUID, read=True, notify=True)

print("Registering services")

# Register services
aioble.register_services(remote_service, device_info)

connected = False

# Function to update the data to be sent to the robot
def updateData():
    controllerOutput = joystickController.readMovements()
    
    # Fits the output of the joystick controller to the format of the data to be sent to the robot
    data[comms.indexSpeedLeftBig] = abs(controllerOutput[1][0])//256
    data[comms.indexSpeedLeftLittle] = abs(controllerOutput[1][0])%256
    
    data[comms.indexSpeedRightBig] = abs(controllerOutput[1][1])//256
    data[comms.indexSpeedRightLittle] = abs(controllerOutput[1][1])%256
    
    data[comms.indexMotorLeftDirection] = controllerOutput[0][0]+1
    data[comms.indexMotorRightDirection] = controllerOutput[0][1]+1

    data[comms.indexButton] = controllerOutput[2]


# Function to update the OLED display
async def updateOLED():
    while True:
        # If there is no connection
        if not connected:
            display.fill(0)
            display.text('Humles Joystick', 0, 0, 1)
            display.text('Ikke forbundet', 0, 12, 1)
            display.show()
            await asyncio.sleep_ms(1000)
            continue
        # If there is a connection
        display.fill(0)
        display.text('Speeds: '+ str(data[comms.indexSpeedLeftLittle]+data[comms.indexSpeedLeftBig]*256) + ' ' + str(data[comms.indexSpeedRightLittle]+data[comms.indexSpeedRightBig]*256), 0, 0, 1)
        display.text('Directions: '+ str(data[comms.indexMotorLeftDirection]-1) + ' ' + str(data[comms.indexMotorRightDirection]-1), 0, 12, 1)
        display.show()
        await asyncio.sleep_ms(1000)




async def remote_task():#
    """ Task to handle remote control """
    
    while True:
        if not connected:
            print("Not Connected")
            await asyncio.sleep_ms(1000)
            continue
        updateData()
        print("Data: ", data)
        joystick_characteristic.write(bytes(data))
        joystick_characteristic.notify(connection, b"x")
        await asyncio.sleep_ms(10)


async def peripheral_task():
    """ Task to handle peripheral """
    global connected, connection
    while True:
        connected = False
        # Start advertising
        async with await aioble.advertise(
            ADV_INTERVAL_MS,
            name="Humles Joystick",
            appearance=_BLE_APPEARANCE_GENERIC_REMOTE_CONTROL,
            services=[_ROBOT]
        ) as connection: # type: ignore
            # When a connection is made
            print("Connection from, ", connection.device)
            connected = True
            print("connected {connected}")
            leds[1].value(1)
            # wait for disconnection
            await connection.disconnected()
            print("disconnected")
            leds[1].value(0)


# Blink the LED
async def blink_task():
    leds[0].value(1)
    await asyncio.sleep_ms(500)

# Main function to run the tasks
async def main():
    tasks = [
        asyncio.create_task(peripheral_task()),
        asyncio.create_task(remote_task()),
        asyncio.create_task(blink_task()),
        asyncio.create_task(updateOLED())
    ]
    await asyncio.gather(*tasks)  # type: ignore

# Run the main function with asyncio
asyncio.run(main())