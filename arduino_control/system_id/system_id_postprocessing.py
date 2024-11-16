import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Step 1: Load the first CSV file into a DataFrame (only first 256 rows)
df1 = pd.read_csv('motor2charac.csv')
df1 = df1.head(256)  # Select first 256 rows

# Step 2: Compute count/s for the first CSV file
df1['count_per_sec'] = df1['encoder'].diff() / df1['time'].diff()

# Step 3: Load the second CSV file into a DataFrame (only first 256 rows)
df2 = pd.read_csv('motor2charac(-1).csv')
df2 = df2.head(256)  # Select first 256 rows

# Step 4: Compute count/s for the second CSV file
df2['count_per_sec'] = df2['encoder'].diff() / df2['time'].diff()
# Step 5: Create a new time vector for the second CSV file, ranging from 50 to 0
new_time_vector = np.linspace(50, 0, num=256)  # Generates 256 equally spaced points from 50 to 0

# Step 6: Plot both datasets on the same graph
plt.figure(figsize=(10, 6))

# Plot the first dataset (df1)
plt.plot(df1['time'], df1['count_per_sec'], linestyle='-', color='b', label='CSV 1')

# Plot the second dataset (df2), using the new time vector
plt.plot(new_time_vector, df2['count_per_sec'], linestyle='-', color='b', label='CSV 2 (Reversed Time)')

# Step 7: Add labels and title
plt.title('Open Loop Motor ID data: motor 2')
plt.xlabel('Time (s)')
plt.ylabel('Count/s')

# Step 8: Display the plot
plt.grid(True)
plt.show()
