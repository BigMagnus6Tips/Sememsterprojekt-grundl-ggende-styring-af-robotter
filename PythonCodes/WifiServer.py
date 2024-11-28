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
            data = wifiAP.conn.recv(6)  # Receive up to 1024 bytes
            if not data:
                break
            print("Received:", list(data))



except Exception as e:
    print(e)
    wifiAP.conn.close()
    wifiAP.server.close()


