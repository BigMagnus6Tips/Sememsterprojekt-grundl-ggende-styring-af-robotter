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

joystickController = JoystickController()



time.sleep(1)

data = [0, 0, 0, 0,]
data[comms.indexMotorRightDirection] = 1
data[comms.indexMotorLeftDirection] = 1
data[comms.indexSpeedLeft] = 100
data[comms.indexSpeedRight] = 100
try:
    while True:
        wifiSTA.client.send(bytes(data))
        print("Sent: ", data)
        time.sleep(1)

    
    
except Exception as e:
    print(e)
    wifiSTA.client.close()