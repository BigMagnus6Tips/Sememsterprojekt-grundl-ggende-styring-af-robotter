from machine import Pin, PWM #  Imports pin and PWM from rasberry pi pico
import uasyncio as asyncio #    Imports asyncio

class ServoMove: #  Defines a ServoMove class
    def __init__(self, pin, frequency=50): #   Initializes a ServoMove object that takes PWM and a frequency    
        self.pwm = PWM(Pin(pin)) #  Creates a PWM pin on the pin we call
        self.pwm.freq(frequency) #  Sets the PWM frequency to specified value
    
    def set_angle(self, angle): #   Method to set the servo to a specific angle between 0 and 180 degrees
        if 0 <= angle <= 180: # Validates that the specified angle is within the correct range
            duty = int(1000 + (angle / 180.0) * 8000) # Here we convert the angle to a PWM duty cycle
            self.pwm.duty_u16(duty) #   Sends the calculated duty cycle to adjust its position
        else:
            raise ValueError("Angle must be between 0 and 180") #   If angle is not between 0 and 180 prints error

    async def servo_move(self, start_angle, end_angle, step_size, delay, task_delay): # An async function to control a servo
        #   
        if step_size <= 0: 
            raise ValueError("Step size must be a positive integer")
        if start_angle >= end_angle:
            raise ValueError("Start angle must be less than end angle")
        
        try:
            while True:
                # Move from start_angle to end_angle
                for current_angle in range(start_angle, end_angle, step_size):
                    self.set_angle(current_angle)
                    await asyncio.sleep(delay)
                    print(f"Moving up: {current_angle}")
                
                # Pause before moving back
                await asyncio.sleep(task_delay)
                
                # Move from end_angle back to start_angle
                for current_angle in range(end_angle, start_angle, -step_size):
                    self.set_angle(current_angle)
                    await asyncio.sleep(delay)
                    print(f"Moving down: {current_angle}")
        except asyncio.keyboardinterupt:
            print("Task cancelled! Returning servo to start position...")
            for current_angle in range(current_angle, start_angle - 1, -step_size):
                self.set_angle(current_angle)
                await asyncio.sleep(delay)
                print(f"Returning to start: {current_angle}")
        finally:
            self.deinit()

    def deinit(self):
        self.pwm.deinit()

class RoboticArm:
    def __init__(self, servo_pins):
        self.servos = [ServoMove(pin) for pin in servo_pins]
    
    async def wave(self):
        # Example of a simple wave movement
        await self.servos[0].servo_move(0, 90, 5, 0.05, 1)

async def main():
    # Example pin configuration
    arm = RoboticArm([15, 16, 17])  # Replace with actual pin numbers
    task = asyncio.create_task(arm.wave())
    await asyncio.sleep(10)  # Run the task for 10 seconds
    task.cancel()  # Cancel the task

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program terminated by user.")

