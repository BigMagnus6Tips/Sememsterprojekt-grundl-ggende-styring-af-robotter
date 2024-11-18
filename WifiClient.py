import network
import time
import socket
from WifiClasses import WifiSTA
"""
Must be nikolais pico because it works
"""
# Wi-Fi credentials
SSID = "NAME"
PASSWORD = "PASSWORD"

wifiSTA = WifiSTA(SSID, PASSWORD)



time.sleep(3)

counter = 0
try:
    while True:
        counter = counter+1
        wifiSTA.client.send(counter.to_bytes(4, 'big'))
        time.sleep(1)
        print("Sent: ", counter)
    
    
except KeyboardInterrupt:
    wifiSTA.client.close()