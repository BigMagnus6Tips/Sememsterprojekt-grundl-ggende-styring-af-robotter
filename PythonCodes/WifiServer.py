import network
from time import sleep
import socket
from WifiClasses import WifiAP, ConstantsForCommunication as comms

"""
must be Karls pico because it works
"""

ssid = "NAME"
password = "PASSWORD"

  
wifiAP = WifiAP(ssid, password)




try:
    while True:
        while True:
            data = wifiAP.conn.recv(1024)  # Receive up to 1024 bytes
            if not data:
                break
            print("Received:", list(data))
            print(data[comms.indexJoystick])
            print(data[comms.indexSpeedLeft])
            print(data[comms.indexSpeedRight])


except Exception as e:
    print(e)
    wifiAP.conn.close()
    wifiAP.server.close()


