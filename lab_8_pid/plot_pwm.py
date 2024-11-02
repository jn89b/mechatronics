import os
import matplotlib.pyplot as plt
import pandas as pd
# go to a folder and get all csv
path = 'm180'
files = os.listdir(path)
files = [f for f in files if f.endswith('.csv')]
print(files)

fig, ax = plt.subplots(4, 1, figsize=(10, 10))
# share the same x axis
fig.tight_layout()
for i, f in enumerate(files):
    df = pd.read_csv(f'{path}/{f}')
    time = df['Time']
    encoder_position = df['EncoderPosition']
    #get the name of the file
    file_name = f.split('.')[0]
    index = 300
    #clip it to 6 seconds
    time = time[:index]    
    encoder_position = encoder_position[:index]
    target_count = df['TargetCounts'][:index]
    ax[i].plot(time, encoder_position, label=file_name + ' Encoder Position')
    ax[i].plot(time, target_count, 'k--', label='Setpoint')
    ax[i].set_xlabel('Time (s)')
    ax[i].set_ylabel('Encoder Position')
    
#ax.set_title('Encoder Position and Target Count vs Time')
for a in ax:
    a.legend()
    
#save the figure
plt.savefig(path+'.png')
plt.show()
plt.close()
