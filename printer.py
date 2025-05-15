from serial import Serial

import time

# Configure serial connection
arduino = Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

# Create a timestamped filename and open a file to write output
timestamp = time.strftime("%Y%m%d-%H%M%S")
filename = f"arduino_output_{timestamp}.txt"
output_file = open(filename, "w")

try:
    print("Reading from serial port. Press Ctrl+C to stop.")
    while True: 
        # Read data from Arduino
        data = arduino.readline()
        
        # Convert bytes to string for better display
        data_str = data.decode('utf-8', errors='replace').strip()
        
        # Only process if there's actual data
        if data_str:
            # Print formatted output to console
            print(f"Received: {data_str}")
            
            # Also write to the file
            output_file.write(f"Received: {data_str}\n")
            output_file.flush()  # Ensure data is written immediately
        
        time.sleep(0.01)  # Small delay to prevent CPU hogging

except KeyboardInterrupt:
    print("Program terminated by user")
finally:
    # Close the file when done
    output_file.close()
    arduino.close()
