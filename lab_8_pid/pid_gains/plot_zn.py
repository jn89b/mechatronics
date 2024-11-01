import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('pid_data.csv')

time = df['Time']
encoder_position = df['EncoderPosition']

fig, ax = plt.subplots()
ax.plot(time, encoder_position, label='Encoder Position')
ax.plot(time, df['TargetCounts'], 'k--', label='Setpoint')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Encoder Position')

plt.show()