import serial
import time
import pandas as pd
import threading
import re

# Serial settings
SERIAL_PORT = 'COM6'  # Update this to your Arduino port
BAUD_RATE = 9600   
serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow time for the serial connection to initialize

# Lists to hold data for saving
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
            match = re.search(r"Encoder Position \| TargetCounts: (\d+\.\d+),(\d+),(-?\d+)", line)
            if match:
                try:
                    # Extract the matched groups as numbers
                    time_sec = float(match.group(1))
                    encoder_position = int(match.group(2))
                    target_count = int(match.group(3))

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

# Keep the script running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")

# Cleanup serial connection on exit
serial_connection.close()
