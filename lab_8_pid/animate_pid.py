import serial
import time
import pandas as pd
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Serial settings
SERIAL_PORT = 'COM6'  # Update this to your Arduino port
BAUD_RATE = 57600   
serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow time for the serial connection to initialize

# Lists to hold data for plotting and saving
time_data = []
encoder_position_data = []
target_count_data = []

# Lock for thread safety
data_lock = threading.Lock()

# Function to read data from the serial port
def read_serial_data():
    while True:
        if serial_connection.in_waiting > 0:
            # Read and decode the serial data
            line = serial_connection.readline().decode('utf-8', errors='replace').strip()
            data = line.split(',')
            if len(data) == 3:
                try:
                    time_sec = float(data[0])
                    encoder_position = int(data[1])
                    target_count = int(data[2])

                    # Append data to lists within the lock
                    with data_lock:
                        time_data.append(time_sec)
                        encoder_position_data.append(encoder_position)
                        target_count_data.append(target_count)

                except ValueError:
                    pass

# Function to save data to CSV periodically
def save_data_to_csv():
    while True:
        time.sleep(5)  # Save every 5 seconds
        with data_lock:
            df = pd.DataFrame({
                "Time": time_data,
                "EncoderPosition": encoder_position_data,
                "TargetCounts": target_count_data
            })
            df.to_csv("encoder_data.csv", index=False)

# Start threads for reading data and saving CSV
threading.Thread(target=read_serial_data, daemon=True).start()
threading.Thread(target=save_data_to_csv, daemon=True).start()

# Setup the live plot
fig, ax = plt.subplots()
ax.set_title("Encoder Position and Target Count vs Time")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Values")

# Plot lines for encoder position and target count
encoder_line, = ax.plot([], [], label="Encoder Position", color="blue")
target_line, = ax.plot([], [], label="Target Count", color="red")  # Explicitly set color to red for target count
ax.legend()

# Initialize plot limits and data range
def init_plot():
    ax.set_xlim(0, 10)  # Initial x-axis limit, will adjust
    ax.set_ylim(-1000, 1000)  # Adjust based on your expected range
    return encoder_line, target_line

# Update the plot in real time
def update_plot(frame):
    with data_lock:
        if time_data:
            # Update line data for encoder position and target count
            encoder_line.set_data(time_data, encoder_position_data)
            target_line.set_data(time_data, target_count_data)
            # Debugging print statements
            # Update x-axis to show the last 10 seconds of data
            max_time = time_data[-1]
            min_time = max(0, max_time - 10)  # Show the last 10 seconds
            ax.set_xlim(min_time, max_time)

            # Adjust y-axis to fit the range of both encoder position and target count
            min_val = min(min(encoder_position_data), min(target_count_data)) - 10
            max_val = max(max(encoder_position_data), max(target_count_data)) + 10
            ax.set_ylim(min_val, max_val)

            ax.relim()
            ax.autoscale_view()

    return encoder_line, target_line

# Animate the plot with FuncAnimation
ani = FuncAnimation(fig, update_plot, init_func=init_plot, blit=False, interval=100)

# Show the plot
plt.show()

# Cleanup serial connection on exit
serial_connection.close()
