import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Step 1: Load the first CSV file into a DataFrame (only first 256 rows)
df1 = pd.read_csv('motor1charac.csv')
df1 = df1.head(256)  # Select first 256 rows

# Step 2: Compute count/s for the first CSV file
df1['count_per_sec'] = df1['encoder'].diff() / df1['time'].diff()

# Step 3: Load the second CSV file into a DataFrame (only first 256 rows)
df2 = pd.read_csv('motor1charac(-1).csv')
df2 = df2.head(256)  # Select first 256 rows

# Step 4: Compute count/s for the second CSV file
df2['count_per_sec'] = df2['encoder'].diff() / df2['time'].diff()
# Step 5: Create a new time vector for the second CSV file, ranging from 50 to 0
new_time_vector = np.linspace(50, 0, num=256)
new_input_vector = np.linspace(-255, 0, num=256)  # Generates 256 equally spaced points from -255 to 0

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

# be serena's bitch
# for state space model, seperately for both motors, u = input (-255 to + 255, o = output (count/s), .csv file with this information
# Step 9: Create a new DataFrame for the combined data
combined_df = pd.DataFrame()

# Add the time and count/s data from the first dataset
combined_df['input voltage'] = df1['input_value']
combined_df['count_per_sec'] = df1['count_per_sec']

# Add the time and count/s data from the second dataset using the new time vector
combined_df_reversed = pd.DataFrame({
    'input voltage': new_input_vector,
    'count_per_sec': df2['count_per_sec'][::-1]
})

# Concatenate both DataFrames in the same order as plotted
result_df = pd.concat([combined_df_reversed, combined_df], ignore_index=True)

# Step 10: Save the combined data to a new CSV file
result_df.to_csv('resultsmotor1.csv', index=False)

print("Data successfully saved to 'combined_motor_data.csv'.")
