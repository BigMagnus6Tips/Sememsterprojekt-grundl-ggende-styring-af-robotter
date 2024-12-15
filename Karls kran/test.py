from machine import Pin, PWM
import uasyncio as asyncio

class ServoMove:
    def __init__(self, pin, allowedRange, startAngle, frequency=50, ):
        """
        Initialize the servo motor with PWM control.
        :param pin: GPIO pin connected to the servo.
        :param frequency: PWM frequency (default 50 Hz for servos).
        """
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(frequency)
        self.angle = startAngle
        self.allowedRange = allowedRange
    def set_angle(self, angle):
        """
        Set the servo angle.
        :param angle: Angle in degrees (0 to 180).
        """
        if self.allowedRange[0] <= angle <= self.allowedRange[1]:
            duty = int(1000 + (angle / 180.0) * 8000)
            self.pwm.duty_u16(duty)
        else:
            raise ValueError(f"Angle must be between {self.allowedRange[0]} and {self.allowedRange[1]}")

    async def servo_move_up(self, start_angle, end_angle, step_size, delay):
        """
        Move the servo back and forth between start_angle and end_angle.
        :param start_angle: Starting angle (0 to 180).
        :param end_angle: Ending angle (0 to 180).
        :param step_size: Increment size for each movement.
        :param delay: Delay between each step (in seconds).
        :param task_delay: Pause before reversing the motion (in seconds).
        """
        if step_size <= 0:
            raise ValueError("Step size must be a positive integer")
        if start_angle >= end_angle:
            raise ValueError("Start angle must be less than end angle")

        try:
            # Move from start_angle to end_angle
            for current_angle in range(start_angle, end_angle, step_size):
                self.set_angle(current_angle)
                await asyncio.sleep(delay)
                print(f"Moving up: {current_angle}")
        finally:
            self.deinit()

    async def servo_move_down(self, start_angle, end_angle, step_size, delay):
    
        if step_size <= 0:
            raise ValueError("Step size must be a positive integer")
        if start_angle >= end_angle:
            raise ValueError("Start angle must be less than end angle")
        
        try:
            for current_angle in range(end_angle, start_angle, -step_size):
                self.set_angle(current_angle)
                await asyncio.sleep(delay)
                print(f"Moving down: {current_angle}")
        
        finally:
            self.deinit()

    async def move_to_angle(self, angle, step_size, delay):
        """
        Move the servo to a specific angle.
        :param angle: Target angle (0 to 180).
        :param step_size: Increment size for each movement.
        :param delay: Delay between each step (in seconds).
        """
        if step_size <= 0:
            raise ValueError("Step size must be a positive integer")

        try:
            # Move to the target angle
            for current_angle in range(self.angle, angle, step_size):
                self.set_angle(current_angle)
                await asyncio.sleep(delay)
                print(f"Moving to: {current_angle}")
        finally:
            print("Something went wrong")


    def deinit(self):
        """
        Deinitialize the PWM to release the GPIO pin.
        """
        self.pwm.deinit()


async def start():
    """
    Create and run servo tasks concurrently.
    """



    """    
    await servo1.servo_move_up(75, 100, 2, 0.05)
    await asyncio.sleep(3)
    await servo2.servo_move_up(10, 12, 2, 0.05)
    await asyncio.sleep(3)
    await servo3.servo_move_up(0, 150, 2, 0.05)
    await asyncio.sleep(5)
    await servo3.servo_move_down(0, 150, 2, 0.05)
    await asyncio.sleep(3)
    await servo2.servo_move_down(10, 12, 2, 0.05)
    await asyncio.sleep(3)
    await servo1.servo_move_down(75, 100, 2, 0.05)
    """
    #asyncio.create_task(servo1.servo_move(75, 100, 2, 0.05, 5))
    #asyncio.create_task(servo2.servo_move(30, 100, 2, 0.05, 5))
    #asyncio.create_task(servo3.servo_move(50, 150, 2, 0.05, 5))


    # Keep the tasks running indefinitely
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Shutting down...")

if __name__ == "__main__":
    startAngles = [75, 30, 50]
    servo1 = ServoMove(15, [0, 180], startAngles[0])
    servo2 = ServoMove(16, [0, 180], startAngles[1])
    servo3 = ServoMove(22, [0, 180], startAngles[2])
    
    tasks = [
    asyncio.create_task(servo1.move_to_angle(startAngles[0], 2, 0.05)),
    asyncio.create_task(servo2.move_to_angle(startAngles[1], 2, 0.05)),
    asyncio.create_task(servo3.move_to_angle(startAngles[2], 2, 0.05))
    ]
    asyncio.gather(*tasks) # type: ignore



    try:
        asyncio.run(start())
    except KeyboardInterrupt:
        tasks = [
        asyncio.create_task(servo1.move_to_angle(startAngles[0], 2, 0.05)),
        asyncio.create_task(servo2.move_to_angle(startAngles[1], 2, 0.05)),
        asyncio.create_task(servo3.move_to_angle(startAngles[2], 2, 0.05))
        ]
        asyncio.gather(*tasks) # type: ignore
        print("Program terminated by user.")
