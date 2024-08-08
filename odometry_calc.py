import reimport re
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timedelta

def extract_odometry(line):
    """
    Extracts the timestamp, X coordinate, and Y coordinate from the given line of odometry data.

    Args:
    line (str): A line of text containing odometry data.

    Returns:
    tuple: A tuple containing the timestamp (datetime object), X coordinate (float), and Y coordinate (float).
           Returns None if the line does not match the expected format.
    """
    match = re.search(r'(\d{2}:\d{2}:\d{2}\.\d{3}) -> .*Odometry - X: ([-\d.]+), Odometry - Y: ([-\d.]+)', line)
    if match:
        time_str = match.group(1)
        x = float(match.group(2))
        y = float(match.group(3))
        time = datetime.strptime(time_str, "%H:%M:%S.%f")
        return time, x, y
    return None

def calculate_velocity(times, x_coords, y_coords):
    """
    Calculates the velocity of the robot at each time step based on the X and Y coordinates.

    Args:
    times (list): A list of timestamps (datetime objects).
    x_coords (list): A list of X coordinates (floats).
    y_coords (list): A list of Y coordinates (floats).

    Returns:
    list: A list of velocities (floats).
    """
    velocities = []
    for i in range(1, len(times)):
        dt = (times[i] - times[i-1]).total_seconds()
        dx = x_coords[i] - x_coords[i-1]
        dy = y_coords[i] - y_coords[i-1]
        distance = np.sqrt(dx**2 + dy**2)
        velocity = distance / dt if dt > 0 else 0
        velocities.append(velocity)
    return velocities

# Lists to store data
times = []
x_coords = []
y_coords = []

# Read and process the odometry data from the file
with open('odometry_data.txt', 'r') as file:
    for line in file:
        data = extract_odometry(line)
        if data:
            time, x, y = data
            times.append(time)
            x_coords.append(x)
            y_coords.append(y)

# Calculate velocities
velocities = calculate_velocity(times, x_coords, y_coords)

# Convert times to seconds from start
start_time = times[0]
times_seconds = [(t - start_time).total_seconds() for t in times[1:]]

# Create the XY plot
plt.figure(figsize=(12, 5))
plt.subplot(121)
plt.plot(x_coords, y_coords, '-b')
plt.title('Robot Path')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid(True)
plt.axis('equal')

# Add start and end markers
plt.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Start')
plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, label='End')
plt.legend()

# Create the Velocity vs Time plot
plt.subplot(122)
plt.plot(times_seconds, velocities, '-r')
plt.title('Robot Velocity over Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (m/s)')
plt.grid(True)

plt.tight_layout()
plt.show()
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timedelta

def extract_odometry(line):
    match = re.search(r'(\d{2}:\d{2}:\d{2}\.\d{3}) -> .*Odometry - X: ([-\d.]+), Odometry - Y: ([-\d.]+)', line)
    if match:
        time_str = match.group(1)
        x = float(match.group(2))
        y = float(match.group(3))
        time = datetime.strptime(time_str, "%H:%M:%S.%f")
        return time, x, y
    return None

def calculate_velocity(times, x_coords, y_coords):
    velocities = []
    for i in range(1, len(times)):
        dt = (times[i] - times[i-1]).total_seconds()
        dx = x_coords[i] - x_coords[i-1]
        dy = y_coords[i] - y_coords[i-1]
        distance = np.sqrt(dx**2 + dy**2)
        velocity = distance / dt if dt > 0 else 0
        velocities.append(velocity)
    return velocities

# Lists to store data
times = []
x_coords = []
y_coords = []

# Read and process the odometry data from the file
with open('odometry_data.txt', 'r') as file:
    for line in file:
        data = extract_odometry(line)
        if data:
            time, x, y = data
            times.append(time)
            x_coords.append(x)
            y_coords.append(y)

# Calculate velocities
velocities = calculate_velocity(times, x_coords, y_coords)

# Convert times to seconds from start
start_time = times[0]
times_seconds = [(t - start_time).total_seconds() for t in times[1:]]

# Create the XY plot
plt.figure(figsize=(12, 5))
plt.subplot(121)
plt.plot(x_coords, y_coords, '-b')
plt.title('Robot Path')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid(True)
plt.axis('equal')

# Add start and end markers
plt.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Start')
plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, label='End')
plt.legend()

# Create the Velocity vs Time plot
plt.subplot(122)
plt.plot(times_seconds, velocities, '-r')
plt.title('Robot Velocity over Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (m/s)')
plt.grid(True)

plt.tight_layout()
plt.show()