import json
import numpy as np
import socket
import time
from scipy.signal import butter, filtfilt

# Sensor sensitivities
ACCEL_SENSITIVITY = 2048.0  # counts per g
GRAVITY = 9.81  # m/s^2

# UDP Configurations
UDP_IP = "127.0.0.1"
UDP_PORT = 51912

# Unity scaling factor (from your mapping)
METERS_TO_UNITY = 0.13350  # Adjust based on your mapping

def gps_to_unity(latitude, longitude):
    """
    Converts GPS coordinates to Unity coordinates using the mapping equations.
    """
    # Mapping constants (from your previous mapping)
    LAT_REF = 37.778369
    LON_REF = -122.390004
    C = 3915.367
    F = 1287.471
    A = 0.13350
    B = -0.000166
    D = -0.000174
    E = 0.13450
    METERS_PER_DEG_LAT = 111000
    AVG_LAT_RAD = np.radians(37.896032)
    METERS_PER_DEG_LON = 111000 * np.cos(AVG_LAT_RAD)

    delta_lat = latitude - LAT_REF
    delta_lon = longitude - LON_REF
    delta_n = delta_lat * METERS_PER_DEG_LAT
    delta_e = delta_lon * METERS_PER_DEG_LON
    x_unity = A * delta_e + B * delta_n + C
    z_unity = D * delta_e + E * delta_n + F
    return x_unity, z_unity

def main():
    # Prompt for the JSON file and takeoff GPS coordinates
    json_file = input("Enter the path to the drone log JSON file: ")
    takeoff_lat = float(input("Enter the takeoff latitude: "))
    takeoff_lon = float(input("Enter the takeoff longitude: "))

    # Load data from JSON file
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Convert raw sensor data to physical units and collect accelerations and timestamps
    acc_list = []
    timestamps = []
    for sample in data:
        # Convert accelerometer counts to m/s^2
        ax_g = sample['accelerometer']['x'] / ACCEL_SENSITIVITY
        ay_g = sample['accelerometer']['y'] / ACCEL_SENSITIVITY
        az_g = sample['accelerometer']['z'] / ACCEL_SENSITIVITY

        acc_mps2 = np.array([ax_g * GRAVITY, ay_g * GRAVITY, az_g * GRAVITY])
        sample['accel_mps2'] = {
            'x': acc_mps2[0],
            'y': acc_mps2[1],
            'z': acc_mps2[2]
        }
        acc_list.append(acc_mps2)
        timestamps.append(sample['timestamp'])

    # Convert timestamps to numpy array
    timestamps = np.array(timestamps)

    # Calculate sampling frequency
    dt_list = np.diff(timestamps)
    fs_list = 1 / dt_list
    fs = np.median(fs_list)  # Use median sampling frequency

    # Apply low-pass filter to accelerations
    def low_pass_filter(data, cutoff_frequency, fs):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff_frequency / nyquist
        b, a = butter(N=1, Wn=normal_cutoff, btype='low', analog=False)
        filtered_data = filtfilt(b, a, data, axis=0)
        return filtered_data

    cutoff_freq = 5  # Hz, adjust as needed
    acc_array = np.array(acc_list)
    acc_filtered = low_pass_filter(acc_array, cutoff_freq, fs)

    # Initialize velocity
    velocity = np.zeros(3)

    # Prepare a list to store velocities
    velocities = []

    # Gravity vector (assuming drone's z-axis is up)
    gravity_vector = np.array([0, 0, GRAVITY])

    for i in range(len(data)):
        if i == 0:
            dt = 0  # First sample, no time difference
        else:
            dt = timestamps[i] - timestamps[i - 1]

        # Get filtered acceleration
        acc = acc_filtered[i]

        # Subtract gravity component
        acc_corrected = acc - gravity_vector

        # Integrate acceleration to get velocity
        velocity += acc_corrected * dt

        # Store velocity (copy to avoid reference issues)
        velocities.append(velocity.copy())

    # Convert velocities to Unity units
    velocities_unity = [v * METERS_TO_UNITY for v in velocities]

    # Compute Unity coordinates for takeoff location
    x_unity_takeoff, z_unity_takeoff = gps_to_unity(takeoff_lat, takeoff_lon)

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Sending velocities over UDP...")

    for i in range(len(data)):
        # Compute dt for Unity
        if i == 0:
            dt = 0  # For the first data point, dt is 0
        else:
            dt = timestamps[i] - timestamps[i - 1]

        packet = {
            'timestamp': data[i]['timestamp'],
            'dt': dt,
            'velocity': {
                'x': float(velocities_unity[i][0]),
                'y': float(velocities_unity[i][1]),
                'z': float(velocities_unity[i][2])
            },
            'position_offset': {
                'x': x_unity_takeoff,
                'z': z_unity_takeoff
            }
        }

        # Serialize packet to JSON
        message = json.dumps(packet)

        # Send over UDP
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

        # Wait for the next timestamp (simulate real-time)
        if i < len(data) - 1:
            dt_sleep = timestamps[i + 1] - timestamps[i]
            if dt_sleep > 0:
                time.sleep(dt_sleep)

    print("Data transmission completed.")

if __name__ == "__main__":
    main()
