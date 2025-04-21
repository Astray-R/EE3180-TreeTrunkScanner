import serial
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# Serial Configuration
SERIAL_PORT = 'COM9'  # Change this based on your actual port
BAUD_RATE = 9600
MAX_POINTS = 5000

# Storage for 3D points
x_data, y_data, z_data = [], [], []

def open_serial():
    """Establishes a serial connection and attempts to auto-reconnect if necessary."""
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to {SERIAL_PORT}")
            return ser
        except serial.SerialException:
            print("Waiting for ESP32 connection...")
            time.sleep(2)

ser = open_serial()

def reset_plot():
    """Clears the 3D plot and resets stored data."""
    global x_data, y_data, z_data
    x_data, y_data, z_data = [], [], []
    print("3D plot reset!")

# Save Point CLoud Data as .ply
def save_point_cloud_to_ply(base_name="point_cloud"):
    index = 1
    filename = f"{base_name}_{index}.ply"
    
    # Increment filename if it already exists
    while os.path.exists(filename):
        index += 1
        filename = f"{base_name}_{index}.ply"

    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(x_data)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y, z in zip(x_data, y_data, z_data):
            f.write(f"{x} {y} {z}\n")
    
    print(f"Point cloud saved to {filename}")

def read_serial():
    """Continuously reads serial data from ESP32 in a separate thread."""
    global x_data, y_data, z_data, ser

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if line == "READY":
                print("ESP32 is active, starting scan...")

            elif line == "SCAN_DONE":
                print("Full scan completed, restarting...")

            elif "RESETTING SYSTEM" in line:
                print("ESP32 is restarting, waiting for reconnection...")

            elif "," in line:
                values = line.split(',')
                if len(values) == 3:
                    try:
                        x, y, z = map(float, values)
                        
                        # Simple noise filtering: Ignore distant outliers
                        if abs(x) > 2000 or abs(y) > 2000 or abs(z) > 2000:
                            continue
                        
                        # Store points with a rolling buffer
                        if len(x_data) >= MAX_POINTS:
                            x_data.pop(0)
                            y_data.pop(0)
                            z_data.pop(0)

                        x_data.append(x)
                        y_data.append(y)
                        z_data.append(z)

                        print(f"Received: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                    except ValueError:
                        print("Received malformed data, skipping...")
        except serial.SerialException:
            print("Serial disconnected, trying to reconnect...")
            ser.close()
            ser = open_serial()

# Start serial reading thread
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

# Initialize 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

def on_key(event):
    """Handles keyboard events to reset the plot or save data."""
    if event.key == 'r':
        reset_plot()
    elif event.key == 's':
        save_point_cloud_to_ply()

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
            ax.set_xlim([0, 800])
            ax.set_ylim([0, 1800])
            ax.set_zlim([min(z_data), max(z_data)])
        
        plt.draw()
        plt.pause(0.1)

# Start plot update loop
update_plot()