from machine import Pin
from time import sleep

inputPin = Pin(15, Pin.IN)

try:
    while True:
        print(inputPin.value())
        sleep(0.1)
except KeyboardInterrupt:
    pass