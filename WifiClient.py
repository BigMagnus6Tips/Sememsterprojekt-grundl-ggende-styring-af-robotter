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

kill_switch 

KillSwitch = False
KillSwitchButton = Pin(22, Pin.IN, Pin.PULL_UP)
KillSwitchButton.irq(trigger=Pin.IRQ_FALLING, handler=interruption_handler)

time.sleep(3)

counter = 0
KillSwitch = 572
try:
    while True:
        counter = counter+1
        wifiSTA.client.send(counter.to_bytes(3, 'big'))
        time.sleep(1)
        print("Sent: ", counter)
    
    
except KeyboardInterrupt:
    wifiSTA.client.close()