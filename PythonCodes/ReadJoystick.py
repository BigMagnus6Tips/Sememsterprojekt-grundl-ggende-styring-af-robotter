from RobotClasses import JoystickController
import uasyncio as asyncio
import machine
import time

joystickController = JoystickController("GP28", "GP27")
data = [0, 0, 0, 0, 0, 0]

while True:
    joystickController.readMovements()
    time.sleep(0.1)