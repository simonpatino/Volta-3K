import serial
import time
import keyboard  # Library to detect keypress

SERIAL_PORT = "COM19"  # Adjust if needed
BAUD_RATE = 9600

# Define headers
HEADER_VOLTA = "Iteration, Time (s), Accel_0, Accel_1, Accel_2, Alt, Press, Euler0, Euler1, Euler2, MaxAlt, Stage #, Sat #, Latitude, Longitude"
HEADER_PYRO = "1,2,3,4,5,6,7,8,9,10"

# Output files
OUTPUT_VOLTA = "output_volta.csv"
OUTPUT_PYRO = "output_pyro.csv"

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
time.sleep(2)  # Wait for Teensy to initialize

# Wait for space key to be pressed
print("Press SPACE to request data...")
keyboard.wait("space")  # Waits until space is pressed
print("Requesting data...")

# Send a byte to trigger the file transmission
ser.write(b'R')

# Read and store Volta.txt data
with open(OUTPUT_VOLTA, "w") as file_volta:
    file_volta.write(HEADER_VOLTA + "\n")  # Write the header first
    while True:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            break  # Stop when there's no more data
        file_volta.write(line + "\n")  # Write to file
        print("[Volta]", line)  # Optional: print to console

# Wait for another space press before pulling pyro.txt
print("Press SPACE to request Pyro data...")
keyboard.wait("space")
print("Requesting Pyro data...")
ser.write(b'P')  # Send a different byte to request pyro.txt

# Read and store Pyro.txt data
with open(OUTPUT_PYRO, "w") as file_pyro:
    file_pyro.write(HEADER_PYRO + "\n")  # Write the header first
    while True:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            break  # Stop when there's no more data
        file_pyro.write(line + "\n")  # Write to file
        print("[Pyro]", line)  # Optional: print to console

print(f"✅ Data saved to {OUTPUT_VOLTA} and {OUTPUT_PYRO}")
ser.close()