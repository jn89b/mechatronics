import serial
import time 
import pandas as pd

class DataLogger():
    """
    Used to read serial data from Arduino and save it to a CSV file.
    
    
    - YOU MUST CHANGE THE DATA_DICT KEYS TO MATCH THE DATA BEING SENT FROM ARDUINO.
    - Update the keys in the data_dict to match the data being sent from Arduino.
    - In the read_data method, update the number of keys to match the number of data values being sent from Arduino.
    
    
    """
    def __init__(self,
                 arduino_port:str="COM4",
                 baud_rate:int=9600,
                 timeout:float=1.0,
                 sleep_time:float=2.0,
                 saved_data_file_name:str="filter_data.csv") -> None:

        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.saved_data_file_name = saved_data_file_name 
        self.ser = serial.Serial(arduino_port
                                    , baud_rate
                                    , timeout=timeout)
        
        time.sleep(sleep_time)
        
        self.data_dict = {
            'Time': [],
            'kf_roll_dg': [],
            'kf_pitch_dg': []
        }
        
    def read_data(self) -> None:
        print("Reading sensor data from Arduino...")
        try:
            num_keys = len(self.data_dict.keys())
            while True:
                if self.ser.in_waiting > 0:
                    arduino_data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    data_values = arduino_data.split(',')
                    
                    ## UPDATE AND ADD KEYS TO MATCH THE DATA BEING SENT FROM ARDUINO
                    if len(data_values) == num_keys + 1:
                        current_time = float(data_values[0])
                        kf_roll = float(data_values[1])
                        kf_pitch = float(data_values[2])
                        print(f"Time: {current_time}, KF Roll: {kf_roll}, KF Pitch: {kf_pitch}")
                        # yaw = float(data_values[3])
                        # accel_x = float(data_values[4])
                        # accel_y = float(data_values[5])
                        # accel_z = float(data_values[6])
                        
                        self.data_dict['Time'].append(current_time)
                        self.data_dict['kf_roll_dg'].append(kf_roll)
                        self.data_dict['kf_pitch_dg'].append(kf_pitch)
                        
                        #print(f"Time: {current_time}, Roll: {roll}, Pitch: {pitch},Yaw: {yaw},AccelX: {accel_x},AccelY: {accel_y},AccelZ: {accel_z}")
                    else:
                        print("length of data_values:", len(data_values))
                        print("Unexpected data format received:", arduino_data)
        except KeyboardInterrupt:
            print("Stopping the script. Saving data...")
            self.save_data(file_name=self.saved_data_file_name)
        finally:
            self.ser.close()
            
    def save_data(self, file_name:str) -> None:
        print("Saving data...")
        data_df = pd.DataFrame(self.data_dict)
        data_df.to_csv(file_name, index=False)
        
        print(f"Data saved to {file_name}")
    
if __name__ == "__main__":
    port_name:str = "COM6"
    baud_rate_val:int = 57600
    file_name:str = "filter_data.csv"
    logger = DataLogger(
        arduino_port=port_name,
        baud_rate=baud_rate_val,
        saved_data_file_name=file_name
    )
    logger.read_data()
    
    