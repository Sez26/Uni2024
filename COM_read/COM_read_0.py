"""
ChatGPT code.
"""

import serial
import time

# Set your COM port and baud rate (these should match your Arduino code)
COM_PORT = 'COM5'   # replace 'COM3' with your actual COM port
BAUD_RATE = 9600    # match the baud rate with your Arduino setup
OUTPUT_FILE = './arduino_output.txt'

# Open serial connection
try:
    with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser, open(OUTPUT_FILE, 'w') as file:
        print("Reading from serial port and saving to file...")
        
        # Continuously read data from the serial port
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()  # Read and decode the line
                print(data)  # Print to console (optional)
                file.write(data + '\n')  # Save to file with newline
                file.flush()  # Ensure data is written to the file immediately
            
            # Optional: Stop after some time or based on condition
            time.sleep(0.1)

except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("Interrupted by user. Exiting...")
