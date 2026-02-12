#!/usr/bin/env python3
"""
MDROBOT Motor Driver Baudrate Detection Tool
Automatically detects the baudrate of MD200T/MD400T/MD750T motor drivers
"""

import serial
import time
import sys
import argparse

# MDROBOT Protocol Constants
MDUI_ID = 184  # PC ID
MDT_ID = 183   # Motor Driver ID
MOTOR_ID = 1   # Motor ID

PID_REQ_PID_DATA = 4
PID_MAIN_DATA = 193

# Supported baudrates
BAUDRATES = [9600, 19200, 38400, 57600, 115200]


def calculate_checksum(data: bytes) -> int:
    """Calculate checksum: (~sum + 1) & 0xFF"""
    return (~sum(data) + 1) & 0xFF


def build_request_packet() -> bytes:
    """Build a status request packet (PID_REQ_PID_DATA)"""
    packet = bytearray()
    packet.append(MDT_ID)       # RMID (Motor driver)
    packet.append(MDUI_ID)      # TMID (PC)
    packet.append(MOTOR_ID)     # Motor ID
    packet.append(PID_REQ_PID_DATA)  # PID
    packet.append(1)            # Data size
    packet.append(PID_MAIN_DATA)  # Request PID_MAIN_DATA

    checksum = calculate_checksum(packet)
    packet.append(checksum)

    return bytes(packet)


def verify_response(data: bytes) -> bool:
    """Verify if response is valid MDROBOT packet"""
    if len(data) < 7:
        return False

    # Check header (should contain MDUI_ID and MDT_ID)
    if data[0] not in (MDUI_ID, MDT_ID) or data[1] not in (MDUI_ID, MDT_ID):
        return False

    # Check motor ID
    if data[2] not in (1, 2):
        return False

    # Verify checksum
    checksum = sum(data) & 0xFF
    if checksum != 0:
        return False

    return True


def test_baudrate(port: str, baudrate: int, timeout: float = 0.5) -> bool:
    """
    Test if motor driver responds at given baudrate

    Args:
        port: Serial port path
        baudrate: Baudrate to test
        timeout: Response timeout in seconds

    Returns:
        True if valid response received
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Send request packet
        packet = build_request_packet()
        ser.write(packet)

        # Wait for response
        time.sleep(0.1)

        # Read response
        response = ser.read(ser.in_waiting or 23)
        ser.close()

        if len(response) > 0:
            print(f"  [{baudrate:6d}] Response: {response.hex(' ')}")
            if verify_response(response):
                return True
            else:
                print(f"           (Invalid packet format)")
        else:
            print(f"  [{baudrate:6d}] No response")

        return False

    except serial.SerialException as e:
        print(f"  [{baudrate:6d}] Error: {e}")
        return False


def detect_baudrate(port: str) -> int:
    """
    Detect motor driver baudrate by trying all supported rates

    Args:
        port: Serial port path

    Returns:
        Detected baudrate or -1 if not found
    """
    print(f"\n{'='*50}")
    print(f"MDROBOT Motor Driver Baudrate Detection")
    print(f"{'='*50}")
    print(f"Port: {port}")
    print(f"Testing baudrates: {BAUDRATES}")
    print(f"{'='*50}\n")

    for baudrate in BAUDRATES:
        print(f"Testing {baudrate} bps...")

        # Try multiple times for reliability
        for attempt in range(3):
            if test_baudrate(port, baudrate):
                print(f"\n{'='*50}")
                print(f"SUCCESS! Detected baudrate: {baudrate} bps")
                print(f"{'='*50}\n")
                return baudrate
            time.sleep(0.1)

    print(f"\n{'='*50}")
    print(f"FAILED: Could not detect baudrate")
    print(f"{'='*50}")
    print("\nPossible reasons:")
    print("  1. Motor driver is not powered on")
    print("  2. Serial cable is not connected properly")
    print("  3. Wrong serial port")
    print("  4. Motor driver uses non-standard baudrate")
    print("")

    return -1


def list_serial_ports():
    """List available serial ports"""
    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found!")
        return

    print("\nAvailable serial ports:")
    print("-" * 40)
    for port in ports:
        print(f"  {port.device}: {port.description}")
    print("")


def main():
    parser = argparse.ArgumentParser(
        description='MDROBOT Motor Driver Baudrate Detection Tool'
    )
    parser.add_argument(
        '-p', '--port',
        type=str,
        default='/dev/ttyUSB0',
        help='Serial port (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '-l', '--list',
        action='store_true',
        help='List available serial ports'
    )
    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        default=None,
        help='Test specific baudrate only'
    )

    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return 0

    # Check if port exists
    try:
        ser = serial.Serial(args.port)
        ser.close()
    except serial.SerialException as e:
        print(f"Error: Cannot open port {args.port}")
        print(f"  {e}")
        print("\nUse -l option to list available ports")
        return 1

    if args.baudrate:
        # Test specific baudrate
        print(f"\nTesting specific baudrate: {args.baudrate} bps")
        if test_baudrate(args.port, args.baudrate):
            print(f"\nSUCCESS! Motor driver responds at {args.baudrate} bps")
            return 0
        else:
            print(f"\nFAILED: No response at {args.baudrate} bps")
            return 1
    else:
        # Auto-detect baudrate
        result = detect_baudrate(args.port)
        return 0 if result > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
