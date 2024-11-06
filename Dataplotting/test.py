from machine import ADC, Pin
import time

# Set up the ADC pin (choose one of GP26, GP27, or GP28 for ADC on Pico W)
adc_pin = ADC(Pin(26))  # GP26 is labeled as ADC0 on Pico

# Reference voltage for the Pico W is typically 3.3V
REFERENCE_VOLTAGE = 3.3
R2 = 2200  # Known resistor value in the voltage divider circuit

def read_voltage():
    # Read raw ADC value (12-bit range: 0 to 4095)
    adc_value = adc_pin.read_u16()  # Returns 16-bit value (0-65535)
    
    # Scale the 16-bit reading to voltage
    voltage = (adc_value / 65535) * REFERENCE_VOLTAGE
    return voltage

# Open the file for writing (or appending) data
with open("data_log.csv", "w") as file:
    # Write the CSV header
    file.write("Time (s), Voltage (V), Resistance (Ohm)\n")

# Main loop
current_time = 0

while True:
    # Read current time, voltage, and calculate resistance
    voltage = read_voltage()
    resistance = round(REFERENCE_VOLTAGE * R2 / voltage - R2)

    # Print the data to the console
    print("Time: {:.2f} s, Voltage: {:.2f} V, Resistance: {} Ohm".format(current_time, voltage, resistance))
    
    # Open file in append mode and write the data
    with open("data_log.csv", "w") as file:
        file.write("{:.2f}, {:.2f}, {}\n".format(current_time, voltage, resistance))

    # Delay between readings
    time.sleep(0.1)
    current_time += 0.1
