# June 2023
# Bluetooth cores specification versio 5.4 (0x0D)
# Bluetooth Remote Control
# Kevin McAleer
# KevsRobot.com

import aioble
import bluetooth
import machine
import uasyncio as asyncio
from WifiClasses import ConstantsForCommunication as comms
# Bluetooth UUIDS can be found online at https://www.bluetooth.com/specifications/gatt/services/

_REMOTE_UUID = bluetooth.UUID(0x1848)
_ENV_SENSE_UUID = bluetooth.UUID(0x1878) 
_REMOTE_CHARACTERISTICS_UUID = bluetooth.UUID(0x2A78)

connected = False
alive = False

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
    global connected
    connected = False
    device = await find_remote()
    if not device:
        print("Robot Remote not found")
        return
    try:
        print("Connecting to", device)
        connection = await device.connect()
        
    except asyncio.TimeoutError:
        print("Timeout during connection")
        return
      
    async with connection:
        print("Connected")
        connected = True
        alive = True
        while True and alive:
            try:
                robot_service = await connection.service(_REMOTE_UUID)
                print(robot_service)
                control_characteristic = await robot_service.characteristic(_REMOTE_CHARACTERISTICS_UUID)
                print(control_characteristic)
            except asyncio.TimeoutError:
                print("Timeout discovering services/characteristics")
                return
            while True:
                if control_characteristic != None:
                    try:
                        data = await control_characteristic.read()
                        listData = list(data)
                        dataFromController = [[listData[comms.indexSpeedLeftBig]*256+listData[comms.indexSpeedLeftLittle], 
                                               listData[comms.indexSpeedRightBig]*256+listData[comms.indexSpeedRightLittle]], 
                                              [listData[comms.indexMotorLeftDirection]-1, listData[comms.indexMotorRightDirection]-1],
                                               listData[comms.indexButton]]
                        print(dataFromController)
                        #print(listData)
                    except:
                        print("Something went wrong")
                else:
                    print('no characteristic')
                await asyncio.sleep_ms(10)

async def main():
    tasks = []
    tasks = [
        asyncio.create_task(peripheral_task()),
    ]
    await asyncio.gather(*tasks) # type: ignore
    
while True:
    asyncio.run(main())