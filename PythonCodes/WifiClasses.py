import network
import socket
import time

class ConstantsForCommunication:
    #Indexes 
    indexJoystick = 0
    indexSpeedLeft = 1
    indexSpeedRight = 2
    
    #Values
    forward = 1
    backward = 2
    clockwise = 3
    counterclockwise = 4
    
    


class WifiSTA:
    def __init__(self, ssid, password):
        self.connect_to_wifi(ssid, password)
        self.makeClient()


    def connect_to_wifi(self, ssid, password):
        # Initialize the station interface
        self.sta = network.WLAN(network.STA_IF)
        self.sta.active(True)

     

        # Connect to the network
        print(f"Connecting to Wi-Fi network: {ssid}")
        self.sta.connect(ssid, password)

        # Wait for connection
        max_retries = 50
        while not self.sta.isconnected() and max_retries > 0:
            print("Connecting...")
            time.sleep(1)
            max_retries -= 1

        # Check the connection status
        if self.sta.isconnected():
            print("Connected successfully!")
            print("Network config:", self.sta.ifconfig())
        else:
            print("Failed to connect to Wi-Fi")

    def makeClient(self):
        # Create a TCP client socket
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_ip = "192.168.4.1"  # IP of the AP
        self.server_port = 8080

        print("Connecting to server...")
        self.client.connect((self.server_ip, self.server_port))
        print("Connected to server.")


class WifiAP:
    def __init__(self, ssid, password):
        self.ssid = ssid
        self.password = password
  
        self.ap = network.WLAN(network.WLAN.IF_AP)
        self.ap.config(essid=ssid, password=password)
        self.ap.active(True)

        while not self.ap.active():
            print("AP active: ", self.ap.active())
        print("AP active: ", self.ap.active())
        
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(('', 8080))  # Bind to all interfaces on port 8080
        self.server.listen(1)  # Listen for one connection
        print("Waiting for a connection...")
        # Accept a connection
        self.conn, self.addr = self.server.accept()
        print("Connected by", self.addr)        
    