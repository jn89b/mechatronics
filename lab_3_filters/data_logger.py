import serial
import time 
import pandas as pd

class DataLogger():
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
            'roll_dg': [],
            'pitch_dg': [],
            'yaw_dg': [],
            'accel_x_normal': [],
            'accel_y_normal': [],
            'accel_z_normal': []
        }
        
    def read_data(self) -> None:
        print("Reading sensor data from Arduino...")
        try:
            while True:
                if self.ser.in_waiting > 0:
                    arduino_data = self.ser.readline().decode('utf-8').strip()
                    data_values = arduino_data.split(',')
                    
                    if len(data_values) == 7:
                        current_time = float(data_values[0])
                        roll = float(data_values[1])
                        pitch = float(data_values[2])
                        yaw = float(data_values[3])
                        accel_x = float(data_values[4])
                        accel_y = float(data_values[5])
                        accel_z = float(data_values[6])
                        
                        self.data_dict['Time'].append(current_time)
                        self.data_dict['roll_dg'].append(roll)
                        self.data_dict['pitch_dg'].append(pitch)
                        self.data_dict['yaw_dg'].append(yaw)
                        self.data_dict['accel_x_normal'].append(accel_x)
                        self.data_dict['accel_y_normal'].append(accel_y)
                        self.data_dict['accel_z_normal'].append(accel_z)
                        
                        print(f"Time: {current_time}, Roll: {roll}, Pitch: {pitch},Yaw: {yaw},AccelX: {accel_x},AccelY: {accel_y},AccelZ: {accel_z}")
                    else:
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
    baud_rate_val:int = 9600    
    file_name:str = "filter_data.csv"
    logger = DataLogger(
        arduino_port=port_name,
        baud_rate=baud_rate_val,
        saved_data_file_name=file_name
    )
    logger.read_data()
    
    