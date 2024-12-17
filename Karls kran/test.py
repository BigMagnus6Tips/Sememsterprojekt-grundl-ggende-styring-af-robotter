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
            self.angle = angle
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
        print(f"moving to angle {angle}")
        if step_size <= 0:
            raise ValueError("Step size must be a positive integer")

        angleDiffrence = angle - self.angle
        # Move to the target angle
        if angleDiffrence <0:
            step_size = -step_size
        for current_angle in range(self.angle,angle, step_size):
            self.set_angle(current_angle)
            await asyncio.sleep(delay)
        
    def deinit(self):
        """
        Deinitialize the PWM to release the GPIO pin.
        """
        self.pwm.deinit()

async def goToStart():
    await servo1.move_to_angle(startAngles[0], 2, 0.05)
    await servo2.move_to_angle(startAngles[1], 2, 0.05)
    await servo3.move_to_angle(startAngles[2], 2, 0.05)
    


async def toggle_electromagnet(pin_number, duration, frequency=1000, duty_cycle=50):
        
        electro = PWM(Pin(pin_number, Pin.OUT))
        electro.freq(frequency)
        electro.duty_u16(0)
        
        duty_u16 = int(duty_cycle / 100 * 65535)


        print("electro on")
        
        electro.duty_u16(duty_u16)

        await asyncio.sleep(duration)

        electro.duty_u16(0)
        
        print("electro off")

async def pick_up_bolt():
    """
    Create and run servo tasks concurrently.
    """
    await asyncio.gather(
        servo1.move_to_angle(100, 1, 0.05),
        servo2.move_to_angle(10, 1, 0.1),
        servo3.move_to_angle(128, 5, 0.05),
        servo4.move_to_angle(50, 1, 0.05)
    ) # type: ignore

    # Add a short delay before the next group of movements
    
    await asyncio.gather(
        toggle_electromagnet(12, 0.5, 1000, 50),
        servo1.move_to_angle(110, 2, 0.05),
    ) # type: ignore
    
    await asyncio.gather(
        toggle_electromagnet(12, 1, 1000, 50),
        servo3.move_to_angle(124, 2, 0.05),
        servo4.move_to_angle(35, 2, 0.05)
    ) # type: ignore
    
    await asyncio.gather(
        toggle_electromagnet(12, 1, 1000, 50),
        servo3.move_to_angle(129, 2, 0.05),
        servo4.move_to_angle(70, 2, 0.05)
    ) # type: ignore
    
    await asyncio.gather(
        toggle_electromagnet(12, 1, 1000, 50),
        servo1.move_to_angle(90, 2, 0.05),
        servo3.move_to_angle(137, 2, 0.05),
        servo4.move_to_angle(45, 2, 0.05)
    ) # type: ignore
    
    await asyncio.gather(
        toggle_electromagnet(12, 0.5, 1000, 50),
        servo3.move_to_angle(120, 2, 0.05)
    ) # type: ignore
#
    await asyncio.gather(
        toggle_electromagnet(12, 0.5, 1000, 50),
        servo1.move_to_angle(60, 5, 0.05)
    ) # type: ignore
    
    await asyncio.gather(
        toggle_electromagnet(12, 1.5, 1000, 50),
        servo3.move_to_angle(10, 5, 0.05),
        servo4.move_to_angle(140, 5, 0.05)
    ) # type: ignore
    
    ## Move servos back to their positions concurrently
    await asyncio.gather(
        servo4.move_to_angle(0, 5, 0.05),
        servo3.move_to_angle(0, 5, 0.05),
        servo2.move_to_angle(30, 5, 0.05),
        servo1.move_to_angle(75, 1, 0.05)
    ) # type: ignore
   
    # Keep the tasks running indefinitely
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Shutting down...")

if __name__ == "__main__":
    startAngles = [75, 30, 0, 0]
    servo1 = ServoMove(8, [0, 180], startAngles[0])
    servo2 = ServoMove(9, [0, 180], startAngles[1])
    servo3 = ServoMove(10, [0, 180], startAngles[2])
    servo4 = ServoMove(11, [0,180], startAngles[3])

    try:
        asyncio.run(goToStart())
        asyncio.run(pick_up_bolt())
    except KeyboardInterrupt:
        asyncio.run(goToStart())
        print("Program terminated by user.")
