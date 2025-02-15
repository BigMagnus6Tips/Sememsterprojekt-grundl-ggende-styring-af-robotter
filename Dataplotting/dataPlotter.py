import matplotlib.pyplot as plt
import csv


# Plot the data
plt.figure(figsize=(10, 5))

# Plot Voltage over Time
THRESHOLD = 0.75


path = "Dataplotting/"
fileNames = ["FinalTestWithoutShield1.csv", "FinalTestWithLED.csv","FinalTestWithLEDAndShield.csv","FinalTestWithoutLEDButShield.csv"]
colors = ["blue", "orange", "green", "red", "purple"]

for i in range(len(fileNames)):
    # Lists to store the data in lists
    time_data = []
    voltage_data = []
    resistance_data = []


    # Read data from CSV file on the Pico
    with open(path+fileNames[i], "r") as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            time_data.append(float(row[0]))        # Time in seconds
            voltage_data.append(float(row[1]))     # Voltage in volts
            resistance_data.append(int(row[2]))    # Resistance in ohms
            
            
    
    plt.subplot(2, 1, 1)
    plt.plot(time_data, voltage_data, label=fileNames[i], color=colors[i])

    # Plot Resistance over Time
    plt.subplot(2, 1, 2)
    plt.plot(time_data, resistance_data, label=fileNames[i], color=colors[i])       



plt.subplot(2, 1, 1)
plt.plot([0,time_data[-1]], [THRESHOLD,THRESHOLD], linestyle="--", color="red", label="Threshold")

plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.title("Voltage over Time")
plt.legend()
plt.grid()




plt.subplot(2, 1, 2)
plt.xlabel("Time (s)")
plt.ylabel("Resistance (ohm)")
plt.title("Resistance over Time")
plt.legend()
plt.grid()

# Show the plots
plt.tight_layout()
plt.show()
