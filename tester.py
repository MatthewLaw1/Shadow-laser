import serial
import serial.tools.list_ports
import struct
import time

# Configuration
PORT = "/dev/tty.usbmodem326C346B31311"  # Replace with your USB port
BAUDRATE = 115200  # Match with Betaflight's MSP baud rate
MSP_HEADER = b"$M<"
MSP_STATUS = 101  # Test MSP command

def list_serial_ports():
    """List all available serial ports."""
    print("Available Serial Ports:")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f" - {port.device}")
    print()
    return [port.device for port in ports]

def checksum(payload):
    """Calculate MSP checksum."""
    return sum(payload) & 0xFF

def build_msp_request(command, payload=[]):
    """Build an MSP request packet."""
    size = len(payload)
    message = MSP_HEADER + bytes([size]) + bytes([command]) + bytes(payload)
    cs = checksum([size, command] + payload)
    return message + bytes([cs])

def check_serial_connection(port, baudrate):
    """Check if the serial port can be opened."""
    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            print(f"✅ Serial connection successful on {port} at {baudrate} baud.")
            return True
    except Exception as e:
        print(f"❌ Failed to open {port} at {baudrate} baud: {e}")
        return False

def send_msp_command(port, baudrate, command):
    """Send an MSP command and print raw response."""
    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            print(f"Sending MSP Command: {command}")
            ser.write(build_msp_request(command))

            # Read raw response
            raw_data = ser.read(50)  # Attempt to read up to 50 bytes
            print(f"Raw Data Received: {raw_data}")

            # Parse the response
            if len(raw_data) < 3 or raw_data[:3] != b"$M>":
                print("❌ Invalid MSP header received.")
                return False
            
            payload_size = raw_data[3]
            command_id = raw_data[4]
            payload = raw_data[5:5+payload_size]
            received_checksum = raw_data[5+payload_size]

            # Validate checksum
            calculated_checksum = checksum([payload_size, command_id] + list(payload))
            if received_checksum != calculated_checksum:
                print(f"❌ Checksum mismatch! Expected: {calculated_checksum}, Received: {received_checksum}")
                return False

            print(f"✅ MSP Response: Command ID: {command_id}, Payload: {payload}")
            return True

    except Exception as e:
        print(f"❌ Error communicating with flight controller: {e}")
        return False

def main():
    # Step 1: List all available ports
    print("🔍 Step 1: Listing all serial ports...")
    ports = list_serial_ports()
    if PORT not in ports:
        print(f"❌ Specified port {PORT} not found in available ports.")
        return

    # Step 2: Check serial connection
    print("🔍 Step 2: Testing serial connection...")
    if not check_serial_connection(PORT, BAUDRATE):
        return

    # Step 3: Send MSP_STATUS command
    print("🔍 Step 3: Sending MSP_STATUS command...")
    if send_msp_command(PORT, BAUDRATE, MSP_STATUS):
        print("✅ MSP_STATUS command successfully processed.")
    else:
        print("❌ Failed to process MSP_STATUS command. Check configuration and setup.")

if __name__ == "__main__":
    main()
