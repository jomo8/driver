import time

# Configuration for the serial port
SERIAL_PORT = '/dev/ttyUSB0'  # Update this with your actual serial port
BAUD_RATE = 9600  # Set this to the baud rate of your UART communication

def main():
    try:
        # Open the serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

        # Send a byte
        byte_to_send = b'\x01'  # Example byte, change as needed
        ser.write(byte_to_send)
        print("Sent:", byte_to_send)

        # Wait for a response (assuming the microcontroller echoes back immediately)
        time.sleep(1)  # Adjust the sleep time based on expected response time

        # Read the response
        while ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print("Received:", response)

    except Exception as e:
        print("Error:", e)
    finally:
        # Close the serial port
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
