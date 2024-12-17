from machine import Pin, PWM
import uasyncio as asyncio

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
        
if __name__ == "__main__":
    
    asyncio.run(toggle_electromagnet(12, 3, 1000, 50))