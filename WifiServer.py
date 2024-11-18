import network
from time import sleep
import socket

"""
must be Karls pico becuase it works
"""


ssid = "NAME"
password = "PASSWORD"

def makeAccesPoint():
        
    ap = network.WLAN(network.WLAN.IF_AP)
    ap.config(essid=ssid, password=password)
    ap.active(True)

    while not ap.active():
        print("AP active: ", ap.active())
    print("AP active: ", ap.active())
    return ap


def makeSocket():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('', 8080))  # Bind to all interfaces on port 8080
    server.listen(1)  # Listen for one connection
    print("Waiting for a connection...")
    # Accept a connection
    conn, addr = server.accept()
    print("Connected by", addr)
    return server, conn


ap = makeAccesPoint()
server, conn = makeSocket()


try:
    while True:
        while True:
            data = conn.recv(1024)  # Receive up to 1024 bytes
            if not data:
                break
            print("Received:", int.from_bytes(data, "big"))

except KeyboardInterrupt:
    conn.close()
    server.close()


