import machine
import time


if __name__ == '__main__':
    
    ldr = machine.Pin(8, machine.Pin.IN, machine.Pin.PULL_UP)
    while True:
        print(ldr.value())
        time.sleep(1)
