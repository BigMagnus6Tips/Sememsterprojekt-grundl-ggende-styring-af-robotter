import network
import time
import socket
from WifiClasses import WifiSTA
from RobotClasses import JoystickController
from machine import Pin
"""
Must be nikolais pico because it works
"""
# Wi-Fi credentials
SSID = "NAME"
PASSWORD = "PASSWORD"

wifiSTA = WifiSTA(SSID, PASSWORD)

joystickController = JoystickController()



time.sleep(1)

counter = 0
try:
    while True:
        counter = counter+1
        wifiSTA.client.send(counter.to_bytes(3, 'big'))
        print("Sent: ", counter)
        time.sleep(1)

    
    
except KeyboardInterrupt:
    wifiSTA.client.close()