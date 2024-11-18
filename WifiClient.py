import network
import time
import socket

"""
Must be nikolajs pico beacuse it works
"""
# Wi-Fi credentials
SSID = "NAME"
PASSWORD = "PASSWORD"

def connect_to_wifi(ssid, password):
    # Initialize the station interface
    sta = network.WLAN(network.STA_IF)
    sta.active(True)

    # Check if already connected
    if sta.isconnected():
        print("Already connected to Wi-Fi")
        print("Network config:", sta.ifconfig())
        return sta

    # Connect to the network
    print(f"Connecting to Wi-Fi network: {ssid}")
    sta.connect(ssid, password)

    # Wait for connection
    max_retries = 50
    while not sta.isconnected() and max_retries > 0:
        print("Connecting...")
        time.sleep(1)
        max_retries -= 1

    # Check the connection status
    if sta.isconnected():
        print("Connected successfully!")
        print("Network config:", sta.ifconfig())
    else:
        print("Failed to connect to Wi-Fi")

    return sta

def makeClient():
    # Create a TCP client socket
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_ip = "192.168.4.1"  # IP of the AP
    server_port = 8080

    print("Connecting to server...")
    client.connect((server_ip, server_port))
    print("Connected to server.")
    return client


# Connect to the Wi-Fi network
sta = connect_to_wifi(SSID, PASSWORD)
client = makeClient()


time.sleep(3)

counter = 0
try:
    while True:
        counter = counter+1
        client.send(counter.to_bytes(4, 'big'))
        time.sleep(1)
        print("Sent: ", counter)
    
    
except KeyboardInterrupt:
    client.close()