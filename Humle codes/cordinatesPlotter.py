import matplotlib.pyplot as plt
from scipy import stats

# Load the coordinates from the CSV files without using pandas
left_coordinates = []
right_coordinates = []

with open('Humle codes/left_coordinates.csv', 'r') as file:
    next(file)  # Skip the first line
    for line in file:
        if line == 'x,y':
            continue
        x, y = map(float, line.strip().split(','))
        left_coordinates.append((x, y))

with open('Humle codes/right_coordinates.csv', 'r') as file:
    next(file)  # Skip the first line
    for line in file:
        if line == 'x,y':
            continue
        x, y = map(float, line.strip().split(','))
        right_coordinates.append((x, y))

# Extract the x and y coordinates
left_x, left_y = zip(*left_coordinates)
right_x, right_y = zip(*right_coordinates)

# Perform linear regression
left_slope, left_intercept, _, _, _ = stats.linregress(left_x, left_y)
right_slope, right_intercept, _, _, _ = stats.linregress(right_x, right_y)

# Save the inclination as a variable
left_inclination = left_slope
right_inclination = right_slope

# Create a plot
plt.figure(figsize=(10, 6))

# Plot the left coordinates
plt.scatter(left_x, left_y, color='blue', label='Left Coordinates')
plt.plot(left_x, [left_slope * x + left_intercept for x in left_x], color='blue', linestyle='--', label=f'Left Linear Fit: y={left_slope:.2f}x+{left_intercept:.2f}')

# Plot the right coordinates
plt.scatter(right_x, right_y, color='red', label='Right Coordinates')
plt.plot(right_x, [right_slope * x + right_intercept for x in right_x], color='red', linestyle='--', label=f'Right Linear Fit: y={right_slope:.2f}x+{right_intercept:.2f}')

# Add labels and legend
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.legend()
plt.title('Coordinates and Linear Regression')

# Show the plot
plt.show()