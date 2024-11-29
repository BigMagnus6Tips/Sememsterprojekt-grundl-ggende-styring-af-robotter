import sys

import aioble
import bluetooth
import machine
import uasyncio as asyncio
from micropython import const
from RobotClasses import JoystickController
from WifiClasses import ConstantsForCommunication as comms
import PythonCodes.ssd1306_OLED as ssd1306_OLED
def uid():
    """ Return the unique id of the device as a string """
    return "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}".format(
        *machine.unique_id())

MANUFACTURER_ID = const(0x02A29)
MODEL_NUMBER_ID = const(0x2A24)
SERIAL_NUMBER_ID = const(0x2A25)
HARDWARE_REVISION_ID = const(0x2A26)
BLE_VERSION_ID = const(0x2A28)

joystickController = JoystickController("GP27", "GP28")
data = [0, 0, 0, 0, 0, 0, 0]
joystickController.addButton("GP22")
leds = [machine.Pin("GP9", machine.Pin.OUT), machine.Pin("GP8", machine.Pin.OUT), machine.Pin("GP7", machine.Pin.OUT), machine.Pin("GP6", machine.Pin.OUT)]

OLEDI2C = machine.I2C(0, scl=machine.Pin(13), sda=machine.Pin(12))
display = ssd1306_OLED.SSD1306_I2C(128, 64, OLEDI2C)

_DEVICE_INFO_UUID = bluetooth.UUID(0x180A) # Device Information
_GENERIC = bluetooth.UUID(0x1848)
_JOYSTICK_UUID = bluetooth.UUID(0x2A78)
_ROBOT = bluetooth.UUID(0x1878)
                              
_BLE_APPEARANCE_GENERIC_REMOTE_CONTROL = const(384)

ADV_INTERVAL_MS = 250_000

device_info = aioble.Service(_DEVICE_INFO_UUID)
                              
connection = None

# Create Characteristic for device info
aioble.Characteristic(device_info, bluetooth.UUID(MANUFACTURER_ID), read=True, initial="KevsRobots.com")
aioble.Characteristic(device_info, bluetooth.UUID(MODEL_NUMBER_ID), read=True, initial="1.0")
aioble.Characteristic(device_info, bluetooth.UUID(SERIAL_NUMBER_ID), read=True, initial=uid())
aioble.Characteristic(device_info, bluetooth.UUID(HARDWARE_REVISION_ID), read=True, initial=sys.version)
aioble.Characteristic(device_info, bluetooth.UUID(BLE_VERSION_ID), read=True, initial="1.0")

remote_service = aioble.Service(_GENERIC)

joystick_characteristic = aioble.Characteristic(
    remote_service, _JOYSTICK_UUID, read=True, notify=True)

print("Registering services")

aioble.register_services(remote_service, device_info)

connected = False

def updateData():
    controllerOutput = joystickController.readMovements()
    updateOLED(controllerOutput)
    #print(controllerOutput)
    data[comms.indexSpeedLeftBig] = abs(controllerOutput[1][0])//256
    data[comms.indexSpeedLeftLittle] = abs(controllerOutput[1][0])%256
    
    data[comms.indexSpeedRightBig] = abs(controllerOutput[1][1])//256
    data[comms.indexSpeedRightLittle] = abs(controllerOutput[1][1])%256
    
    data[comms.indexMotorLeftDirection] = controllerOutput[0][0]+1
    data[comms.indexMotorRightDirection] = controllerOutput[0][1]+1

    data[comms.indexButton] = controllerOutput[2]

def updateOLED(controllerOutput):
    display.fill(0)
    display.text('Speeds: '+ (controllerOutput[1]), 40, 0, 1)
    display.text('Directions: '+ (controllerOutput[0]), 40, 0, 1)
    display.show()




async def remote_task():
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
        """if button_a.read():
            print(f"Button A pressed, connection is: {connection}")
            joystick_characteristic.write(b"a")
            joystick_characteristic.notify(connection, b"a")
        elif button_b.read():
            print(f"Button B pressed, connection is: {connection}")
            joystick_characteristic.write(b"b")
            joystick_characteristic.notify(connection, b"b")
        elif button_x.read():
            print(f"Button X pressed, connection is: {connection}")
            joystick_characteristic.write(b"x")
            joystick_characteristic.notify(connection, b"x")
        elif button_y.read():
            print(f"Button Y pressed, connection is: {connection}")
            joystick_characteristic.write(b"y")
            joystick_characteristic.notify(connection, b"y")
        else:
            joystick_characteristic.write(b"!")
#             button_characteristic.notify(connection, b"!")"""
        await asyncio.sleep_ms(10)

async def peripheral_task():
    """ Task to handle peripheral """
    global connected, connection
    while True:
        connected = False
        async with await aioble.advertise(
            ADV_INTERVAL_MS,
            name="Humles Joystick",
            appearance=_BLE_APPEARANCE_GENERIC_REMOTE_CONTROL,
            services=[_ROBOT]
        ) as connection: # type: ignore
            print("Connection from, ", connection.device)
            connected = True
            print("connected {connected}")
            leds[1].value(1)
            await connection.disconnected()
            print("disconnected")
            leds[1].value(0)

async def blink_task():
    
    leds[0].value(1)

    

    await asyncio.sleep_ms(500)

async def main():
    tasks = [
        asyncio.create_task(peripheral_task()),
        asyncio.create_task(remote_task()),
        asyncio.create_task(blink_task()),
    ]
    await asyncio.gather(*tasks)  # type: ignore

asyncio.run(main())