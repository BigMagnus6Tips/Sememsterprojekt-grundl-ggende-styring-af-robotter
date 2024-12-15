from machine import Pin, ADC
from time import sleep

inputPin = Pin(27, Pin.IN, pull=Pin.PULL_DOWN)
#inputPinADC = ADC(Pin(26))
try:
    while True:
        #voltage = inputPinADC.read_u16() * 3.3 / 65535
        print(inputPin.value())
        sleep(0.1)
except KeyboardInterrupt:
    pass