import sys
sys.path.insert(0, 'packages')

import serial
import struct
import time

# Replace this with your actual serial port and baud rate
SERIAL_PORT = "COM3"  # Update this to match your device
BAUD_RATE = 115200

# MSP Command IDs
MSP_RAW_IMU = 102  # Command ID for MSP_RAW_IMU

# Scaling factors


# Function to calculate the checksum
def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

# Function to send MSP command
def send_msp_command(ser, command_id, payload=None):
    if payload is None:
        payload = []

    size = len(payload)
    header = b'$M<'  # MSP header
    message = bytearray([size, command_id] + payload)
    checksum = calculate_checksum(message)
    packet = header + message + bytes([checksum])

    ser.write(packet)

# Function to read and parse MSP response
def read_msp_response(ser):
    while True:
        if ser.read(1) == b'$':  # Start of frame
            if ser.read(1) == b'M':
                direction = ser.read(1)
                if direction == b'>':  # Response from flight controller
                    size = ser.read(1)[0]
                    command = ser.read(1)[0]
                    data = ser.read(size)
                    checksum = ser.read(1)[0]

                    # Validate checksum
                    calculated_checksum = calculate_checksum(
                        bytearray([size, command]) + data
                    )
                    if checksum == calculated_checksum:
                        return command, data
                    else:
                        print("Checksum mismatch!")
                        return None, None

# Parse MSP_RAW_IMU data
def parse_msp_raw_imu(data):
    if len(data) != 18:
        print("Invalid data length for MSP_RAW_IMU")
        return None

    # Unpack 9 integers (2 bytes each) from the raw data
    imu_values = struct.unpack("<hhhhhhhhh", data)

    # Acceleration (ax, ay, az), Gyroscope (gx, gy, gz), and Magnetometer (mx, my, mz)
    ax, ay, az, gx, gy, gz, mx, my, mz = imu_values

    imu_data = {
        "acceleration": {
            "x": ax,
            "y": ay,
            "z": az,
        },
        "gyroscope": {
            "x": gx,
            "y": gy,
            "z": gz,
        }
    }
    return imu_data

def main():
    try:
        # Open serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

        while True:
            # Send MSP_RAW_IMU command
            send_msp_command(ser, MSP_RAW_IMU)
            #time.sleep(0.001)  # Short delay to allow response

            # Read and parse response
            command, data = read_msp_response(ser)
            if command == MSP_RAW_IMU and data is not None:
                imu_data = parse_msp_raw_imu(data)
                if imu_data:
                    print(f" Acceleration (g): {imu_data['acceleration']} \n Gyroscope (°/s): {imu_data['gyroscope']}", end = "")
                    #print()
                    imu_data["timestamp"] = time.time()
                    output_file = "imu_data.json"
                    with open(output_file, "a") as file:
                        json.dump(imu_data, file, indent=4)
            else:
                print("No valid data received. Retrying...")

    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser:
            ser.close()


if __name__ == "__main__":
    main()