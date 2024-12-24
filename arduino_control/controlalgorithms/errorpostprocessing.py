# input large data table with reference and errors (arduino serial monitor)
# ouput graph showing average error at position on shape

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# data = pd.read_csv('data_collection/111124_triangle_flat.csv')
data = pd.read_csv('C:\\Users\\Izzy Popiolek\\Documents\\GitHub\\Uni2024_MVNLC\\arduino_control\\controlalgorithms\\data_collection\\111124_square_vertical.csv')


# # Extract the first two columns
# first_two_columns = data.iloc[:, :2].values

# # Detect the repeating pattern
# def find_repeating_pattern(arr):
#     length = len(arr)
#     for pattern_len in range(1, length // 2 + 1):
#         pattern = arr[:pattern_len]
#         # Check if the array can be divided into sections of this pattern
#         if all((arr[i:i + pattern_len] == pattern).all() for i in range(0, length, pattern_len)):
#             if length % pattern_len == 0:
#                 return pattern, length // pattern_len
#     return None, 0

# pattern, repetitions = find_repeating_pattern(first_two_columns)

# # Display the result
# if pattern is not None:
#     print("Pattern found:", pattern)
#     print("Number of full repetitions:", repetitions)
# else:
#     print("No repeating pattern found.")


# find the average error at matching positions (e.g. if the reference values are the same)
# every 1000 reference value add error1 and error2 and divide by counter
# for i=1:1:len(repetitions)
#     error1=0
#     error2=0

# Columns we want to perform operations on
col1 = 'error1'
col2 = 'error2'
repetitions = 12

# Initialize an empty matrix with shape (1000, 2) for the results
result_matrix = np.zeros((1000, 2))
# Loop over the first 1000 rows
for i in range(0, 1000):
    # Generate indices dynamically based on max_offset
    indices = [i + 1000 * j for j in range(repetitions)]
    
    # Extract values at generated indices for each column
    values_col1 = data.loc[indices, col1]
    values_col2 = data.loc[indices, col2]

    # Sum the values and divide by the number of offsets (2 * (max_offset + 1))
    result_matrix[i, 0] = values_col1.sum() / repetitions
    result_matrix[i, 1] = values_col2.sum() / repetitions



# plot average error on a graph which has marker for significant positions (corners)
# Convert the result matrix into a DataFrame for easier handling
result_df = pd.DataFrame(result_matrix, columns=[f'{col1}_Avg', f'{col2}_Avg'])

# Plotting
plt.figure(figsize=(10, 6))

# Plot each column with different colors
plt.plot(range(1, 1001), result_df[f'{col1}_Avg'], label=f'{col1} Average', color='blue', linestyle='-')
plt.plot(range(1, 1001), result_df[f'{col2}_Avg'], label=f'{col2} Average', color='red', linestyle='-')

# Labeling the plot
plt.title('Averaged Error for Motor 1 and Motor 2 drawing 12 squares (vertical arm)')
plt.xlabel('Reference Value (1 to 1000)')
plt.ylabel('Average Error')
plt.legend()
# Customizing grid lines
plt.grid(True, which='both', linestyle='--', linewidth=0.5)  # Turn on grid for both major and minor ticks
plt.minorticks_on()  # Enable minor ticks for more x-axis granularity

# Show the plot
plt.show()
