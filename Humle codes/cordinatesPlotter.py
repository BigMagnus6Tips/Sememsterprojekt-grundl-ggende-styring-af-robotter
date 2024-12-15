import matplotlib.pyplot as plt

# Load the coordinates from the CSV files without using pandas
left_coordinates = []
right_coordinates = []

with open('Humle codes/left_coordinates.csv', 'r') as file:
    for line in file:
        if line.startswith('x,y'):  # Skip the first line
            continue
        x, y = map(float, line.strip().split(','))
        left_coordinates.append((x, y))

with open('Humle codes/right_coordinates.csv', 'r') as file:
    for line in file:
        if line.startswith('x,y'): # Skip the first line
            continue
        x, y = map(float, line.strip().split(','))
        right_coordinates.append((x, y))

# Extract the x and y coordinates
left_x, left_y = zip(*left_coordinates)
right_x, right_y = zip(*right_coordinates)

# Create a plot
plt.figure(figsize=(10, 6))

# Plot the left coordinates
plt.scatter(left_x, left_y, color='blue', label='Left Coordinates')

# Plot the right coordinates
plt.scatter(right_x, right_y, color='red', label='Right Coordinates')

# Add labels and title
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Coordinates Plotter')
plt.legend()

# Show the plot
plt.grid(True)
plt.show()