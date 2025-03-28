import requests
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ESP32 web server details
ESP32_IP = "192.168.1.100"  # Replace with your ESP32's IP address
URL = f"http://{ESP32_IP}/data"

# Storage for 3D points
x_data, y_data, z_data = [], [], []

def reset_plot():
    """Clears the 3D plot and resets stored data."""
    global x_data, y_data, z_data
    x_data, y_data, z_data = [], [], []
    print("3D plot reset!")

def fetch_data():
    """Fetches data from the ESP32 web server."""
    global x_data, y_data, z_data

    while True:
        try:
            response = requests.get(URL)
            if response.status_code == 200:
                data = response.json()  # Parse JSON response
                x, y, z = data["x"], data["y"], data["z"]

                # Simple noise filtering: Ignore distant outliers
                if abs(x) > 2000 or abs(y) > 2000 or abs(z) > 2000:
                    continue

                # Store points with a rolling buffer
                if len(x_data) >= 5000:
                    x_data.pop(0)
                    y_data.pop(0)
                    z_data.pop(0)

                x_data.append(x)
                y_data.append(y)
                z_data.append(z)

                print(f"Received: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            else:
                print("Failed to fetch data. Status code:", response.status_code)
        except Exception as e:
            print("Error:", e)
        time.sleep(0.1)  # Adjust delay as needed

# Start data fetching thread
data_thread = threading.Thread(target=fetch_data, daemon=True)
data_thread.start()

# Initialize 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

def on_key(event):
    """Handles keyboard events to reset the plot."""
    if event.key == 'r':
        reset_plot()

fig.canvas.mpl_connect('key_press_event', on_key)

def update_plot():
    """Continuously updates the 3D scatter plot with adaptive scaling."""
    while True:
        ax.clear()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        if x_data and y_data and z_data:
            ax.scatter(x_data, y_data, z_data, c='b', marker='o', s=2)  # s: sets dot size

            # Auto-scaling based on data range
            ax.set_xlim([min(x_data), max(x_data)])
            ax.set_ylim([min(y_data), max(y_data)])
            ax.set_zlim([min(z_data), max(z_data)])

        plt.draw()
        plt.pause(0.1)

# Start plot update loop
update_plot()
