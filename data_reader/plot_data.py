import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
df = pd.read_csv("filter_data.csv")

diff_time = np.diff(df['Time'])
print("Time difference between samples in seconds:", diff_time)

plt.plot(diff_time)
# plt.plot(df['Time'], df['roll_dg'], label='Roll')   

plt.show()