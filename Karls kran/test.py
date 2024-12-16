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
    def setAngle(self, angle):
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

    async def servoMoveUp(self, start_angle, end_angle, step_size, delay):
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
                self.setAngle(current_angle)
                await asyncio.sleep(delay)
        finally:
            self.deinit()

    async def servoMoveDown(self, start_angle, end_angle, step_size, delay):
    
        if step_size <= 0:
            raise ValueError("Step size must be a positive integer")
        if start_angle >= end_angle:
            raise ValueError("Start angle must be less than end angle")
        
        try:
            for current_angle in range(end_angle, start_angle, -step_size):
                self.setAngle(current_angle)
                await asyncio.sleep(delay)
                print(f"Moving down: {current_angle}")
        
        finally:
            self.deinit()

    async def moveToAngle(self, angle, step_size, delay):
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
            self.setAngle(current_angle)
            await asyncio.sleep(delay)
        
    def deinit(self):
        """
        Deinitialize the PWM to release the GPIO pin.
        """
        self.pwm.deinit()

class Crane:
    def __init__(self, servo1, servo2, servo3, servo4, startAngles = [75, 30, 0, 0]):
        self.servo1 = servo1
        self.servo2 = servo2
        self.servo3 = servo3
        self.servo4 = servo4
        self.startAngles = startAngles

    async def goToStart(self):
        await self.servo1.moveToAngle(self.startAngles[0], 2, 0.05)
        await self.servo2.moveToAngle(self.startAngles[1], 2, 0.05)
        await self.servo3.moveToAngle(self.startAngles[2], 2, 0.05)
        await self.servo4.moveToAngle(self.startAngles[3], 2, 0.05)

    async def toggleElectromagnet(self, pin_number, duration, frequency=1000, duty_cycle=50):
        
        electro = PWM(Pin(pin_number, Pin.OUT))
        electro.freq(frequency)
        electro.duty_u16(0)
        
        duty_u16 = int(duty_cycle / 100 * 65535)


        print("electro on")
        
        electro.duty_u16(duty_u16)

        await asyncio.sleep(duration)

        electro.duty_u16(0)
        
        print("electro off")

    async def pickUpBolt(self):
        """
        Create and run servo tasks concurrently.
        """
        await asyncio.gather(
            self.servo1.moveToAngle(100, 1, 0.05),
            self.servo2.moveToAngle(10, 1, 0.1),
            self.servo3.moveToAngle(128, 5, 0.05),
            self.servo4.moveToAngle(50, 1, 0.05)
        ) # type: ignore

        # Add a short delay before the next group of movements
        
        await asyncio.gather(
            self.toggleElectromagnet(16, 0.5, 1000, 50),
            self.servo1.moveToAngle(110, 2, 0.05),
        ) # type: ignore
        
        await asyncio.gather(
            self.toggleElectromagnet(16, 1, 1000, 50),
            self.servo3.moveToAngle(124, 2, 0.05),
            self.servo4.moveToAngle(35, 2, 0.05)
        ) # type: ignore
        
        await asyncio.gather(
            self.toggleElectromagnet(16, 1, 1000, 50),
            self.servo3.moveToAngle(129, 2, 0.05),
            self.servo4.moveToAngle(70, 2, 0.05)
        ) # type: ignore
        
        await asyncio.gather(
            self.toggleElectromagnet(16, 1, 1000, 50),
            self.servo1.moveToAngle(90, 2, 0.05),
            self.servo3.moveToAngle(137, 2, 0.05),
            self.servo4.moveToAngle(45, 2, 0.05)
        ) # type: ignore
        
        await asyncio.gather(
            self.toggleElectromagnet(16, 0.5, 1000, 50),
            self.servo3.moveToAngle(120, 2, 0.05)
        ) # type: ignore
        await asyncio.gather(
            self.toggleElectromagnet(16, 0.5, 1000, 50),
            self.servo1.moveToAngle(60, 5, 0.05)
        ) # type: ignore
        
        await asyncio.gather(
            self.toggleElectromagnet(16, 1.5, 1000, 50),
            self.servo3.moveToAngle(10, 5, 0.05),
            self.servo4.moveToAngle(140, 5, 0.05)
        ) # type: ignore
        
        ## Move servos back to their positions concurrently
        await asyncio.gather(
            self.servo4.moveToAngle(0, 5, 0.05),
            self.servo3.moveToAngle(0, 5, 0.05),
            self.servo2.moveToAngle(30, 5, 0.05),
            self.servo1.moveToAngle(75, 1, 0.05)
        ) # type: ignore
    
        # Keep the tasks running indefinitely
        #try:
        #    while True:
        #        await asyncio.sleep(1)
        #except asyncio.CancelledError:
        #    print("Shutting down...")

if __name__ == "__main__":
    startAngles = [75, 30, 0, 0]
    servo1 = ServoMove(8, [0, 180], startAngles[0])
    servo2 = ServoMove(9, [0, 180], startAngles[1])
    servo3 = ServoMove(12, [0, 180], startAngles[2])
    servo4 = ServoMove(11, [0,180], startAngles[3])

    crane = Crane(servo1, servo2, servo3, servo4)
    try:
        asyncio.run(crane.goToStart())
        asyncio.run(crane.pickUpBolt())
    except KeyboardInterrupt:
        asyncio.run(crane.goToStart())
        print("Program terminated by user.")
