import network
import time
import socket
from WifiClasses import WifiSTA
from RobotClasses import JoystickController
from machine import Pin
from WifiClasses import ConstantsForCommunication as comms
"""
Must be nikolais pico because it works
"""
# Wi-Fi credentials
SSID = "NAME"
PASSWORD = "PASSWORD"

wifiSTA = WifiSTA(SSID, PASSWORD)

joystickController = JoystickController("GP28", "GP27")



time.sleep(1)

data = [0, 0, 0, 0, 0, 0]

def updateData():
    controllerOutput = joystickController.joystickMove()
    data[comms.indexSpeedLeftBig] = controllerOutput[1][0]//256
    data[comms.indexSpeedLeftLittle] = controllerOutput[1][0]%256
    
    data[comms.indexSpeedRightBig] = controllerOutput[1][1]//256
    data[comms.indexSpeedRightLittle] = controllerOutput[1][1]%256
    
    data[comms.indexMotorLeftDirection] = controllerOutput[0][0]+1
    data[comms.indexMotorRightDirection] = controllerOutput[0][1]+1


try:
    while True:
        updateData()
        wifiSTA.client.send(bytes(data))
        print("Sent: ", data)
        time.sleep(0.05)

    
    
except Exception as e:
    print(e)
    wifiSTA.client.close()