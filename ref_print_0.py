"""
This is a function to print timeseries refernce signals to a .h file to be imported
into the .ino code

Lots of this was helped by chatGPT
"""

import os

def print_ref(save_dir, file_name, ref):

    # Construct the full file path
    file_path = os.path.join(save_dir, file_name)
    # Open the header file for writing
    ref_t = ref[:,0]
    ref_th_1 = ref[:,1]
    ref_th_2 = ref[:,2]
    with open(file_path, 'w') as f:
        # Write the C++ array declaration for the x and y coordinates
        f.write('#ifndef TIME_SERIES_DATA_H\n')
        f.write('#define TIME_SERIES_DATA_H\n\n')
        
        # Declare t, x and y arrays
        f.write('const int ref_t[] = {')
        f.write(', '.join(map(str, ref_t)))  # Join x values as a string
        f.write('};\n')

        f.write('const int x_coords[] = {')
        f.write(', '.join(map(str, ref_th_1)))  # Join x values as a string
        f.write('};\n')
        
        f.write('const int y_coords[] = {')
        f.write(', '.join(map(str, ref_th_2)))  # Join y values as a string
        f.write('};\n\n')
        
        # Define the size of the arrays
        f.write(f'const int num_points = {len(ref)};\n\n')
        
        # End the header guard
        f.write('#endif // TIME_SERIES_DATA_H\n')