import serial
import time
import os
import sys

# Configuration
# DEFAULT_COM_PORT = 'COM15'  # Default COM port
DEFAULT_COM_PORT = 'COM25'  # Adjusted default to a more common one, but user should specify
BAUD_RATE = 115200
FIRMWARE_FILE = 'test_payload.bin'

def ota_update(port, file_path):
    print(f"Starting OTA Test on {port} with {file_path}...")
    try:
        # Create dummy file if not exists
        if not os.path.exists(file_path):
            print(f"Creating dummy test file {file_path} (1KB)...")
            with open(file_path, 'wb') as f:
                # Write recognizable pattern
                for i in range(1024):
                    f.write(bytes([i % 256]))

        file_size = os.path.getsize(file_path)
        print(f"Opening {port} at {BAUD_RATE}...")
        
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=2)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return

        with ser:
            # 1. Send OTA_START command
            cmd = f"OTA_START:{file_size}\n"
            print(f"Sending command: {cmd.strip()}")
            ser.write(cmd.encode())
            
            # Wait for response (loop until OK or timeout)
            print("Waiting for response...")
            start_wait = time.time()
            ota_started = False
            
            while time.time() - start_wait < 5:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                print(f"Device response: {line}")
                if "OK: OTA Transfer Started" in line:
                    ota_started = True
                    break
                if "ERR" in line:
                    print(f"Error received: {line}")
                    return

            if not ota_started:
                print("Error: Device did not accept OTA start or timed out.")
                return

            print("OTA Start confirmed. Proceeding...")

            time.sleep(1) # Give device time to prepare file

            # 2. Send file content
            print(f"Sending {file_size} bytes...")
            with open(file_path, 'rb') as f:
                chunk_size = 64 # Send in small chunks to avoid overwhelming buffer
                sent = 0
                while True:
                    data = f.read(chunk_size)
                    if not data:
                        break
                    ser.write(data)
                    sent += len(data)
                    # print(f"Sent {sent}/{file_size}", end='\r')
                    time.sleep(0.005) # Small delay to be safe
            
            print(f"\nSent {sent} bytes.")
            
            # 3. Wait for completion message
            print("Waiting for completion...")
            transfer_complete = False
            start_wait = time.time()
            while time.time() - start_wait < 10:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"Device: {line}")
                if "Transfer Complete" in line:
                    print("SUCCESS: OTA Transfer Verified!")
                    transfer_complete = True
                    break
            else:
                print("Timeout waiting for completion confirmation.")
                
            if transfer_complete:
                print("Sending OTA_APPLY to reboot...")
                ser.write(b"OTA_APPLY\n")
                # Wait for reset message
                time.sleep(1)
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"Device: {line}")

    except Exception as e:
        print(f"Unexpected Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = DEFAULT_COM_PORT
        print(f"Using default port {port}. Usage: python ota_sender.py <COM_PORT> [FILE_PATH]")
    
    file_path = FIRMWARE_FILE
    if len(sys.argv) > 2:
        file_path = sys.argv[2]
        
    ota_update(port, file_path)
