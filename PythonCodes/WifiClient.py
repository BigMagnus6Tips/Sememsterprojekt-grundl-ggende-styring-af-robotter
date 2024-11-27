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

data = [0, 0, 0, 0,]

def updateData():
    controllerOutput = joystickController.joystickMove()
    data[comms.indexSpeedLeft] = controllerOutput[1][0]
    data[comms.indexSpeedRight] = controllerOutput[1][1]
    data[comms.indexMotorLeftDirection] = controllerOutput[0][0]
    data[comms.indexMotorRightDirection] = controllerOutput[0][1]


try:
    while True:
        wifiSTA.client.send(bytes(data))
        print("Sent: ", data)
        time.sleep(1)

    
    
except Exception as e:
    print(e)
    wifiSTA.client.close()