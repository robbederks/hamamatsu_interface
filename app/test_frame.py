#!/usr/bin/env python3
"""
Test script for Hamamatsu C7921CA-02 frame acquisition debugging.
Outputs frame statistics to help diagnose transfer issues.
"""

import usb1
import time
import struct
import sys

# Constants
VID = 0x16c0
PID = 0x0483
CUSTOM_INTERFACE = 2
CONTROL_OUT_ENDPOINT = 5
CONTROL_IN_ENDPOINT = 6
BULK_IN_ENDPOINT = 7

FRAME_WIDTH = 1056
FRAME_HEIGHT = 1056
FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT * 2  # 2,230,272 bytes


def connect():
    handle = usb1.USBContext().openByVendorIDAndProductID(VID, PID)
    if handle is None:
        print("ERROR: Could not find Teensy. Make sure it's connected.")
        sys.exit(1)
    handle.claimInterface(CUSTOM_INTERFACE)
    print(f"Connected to Teensy (VID={VID:#06x}, PID={PID:#06x})")
    return handle


def command(handle, cmd, data=b""):
    """Send command and receive response."""
    handle.bulkWrite(CONTROL_OUT_ENDPOINT, struct.pack("<BI", cmd, len(data)) + data)
    resp = handle.bulkRead(CONTROL_IN_ENDPOINT, 512, timeout=1000)
    if len(resp) < 4:
        raise Exception(f"Invalid response length: {len(resp)}")
    resp_len = struct.unpack("<I", resp[:4])[0]
    resp = resp[4:]
    while len(resp) < resp_len:
        resp += handle.bulkRead(CONTROL_IN_ENDPOINT, resp_len - len(resp), timeout=1000)
    return resp


def ping(handle):
    """Test connectivity."""
    dat = command(handle, 0x00)
    if len(dat) != 1 or dat[0] != 0xA5:
        print(f"ERROR: Ping failed, got {dat.hex()}")
        return False
    print("Ping: OK")
    return True


def get_state(handle):
    """Get firmware state."""
    dat = command(handle, 0x01)
    state, row, col = struct.unpack("<BII", dat)
    state_names = {0: "IDLE", 1: "WAITING_ON_VSYNC", 2: "ACQUIRING", 3: "DONE"}
    print(f"State: {state_names.get(state, state)}, row={row}, col={col}")
    return state, row, col


def start_trigger(handle):
    """Start frame acquisition."""
    dat = command(handle, 0x03)
    if dat[0] != 0:
        print("ERROR: Failed to start trigger")
        return False
    print("Trigger started")
    return True


def get_frame(handle, timeout_ms=10000):
    """Get frame data with detailed diagnostics."""
    # Request frame transfer
    dat = command(handle, 0x02)
    if len(dat) != 4:
        print(f"ERROR: Unexpected response length: {len(dat)}")
        return None

    frame_len = struct.unpack("<I", dat[:4])[0]
    print(f"Frame length reported: {frame_len} bytes (expected {FRAME_BYTES})")

    # Read bulk data with timeout
    start = time.time()
    frame_data = b""
    read_count = 0

    while len(frame_data) < frame_len:
        remaining = frame_len - len(frame_data)
        try:
            chunk = handle.bulkRead(BULK_IN_ENDPOINT, remaining, timeout=timeout_ms)
            read_count += 1
            frame_data += chunk
            if len(chunk) == 0:
                print(f"WARNING: Zero-length read at {len(frame_data)} bytes")
                break
        except usb1.USBErrorTimeout:
            print(f"TIMEOUT after {len(frame_data)} bytes ({read_count} reads)")
            break
        except Exception as e:
            print(f"ERROR: {e} after {len(frame_data)} bytes")
            break

    elapsed = time.time() - start
    print(f"Read {len(frame_data)} bytes in {read_count} reads ({elapsed:.2f}s)")
    print(f"Transfer rate: {len(frame_data) / elapsed / 1e6:.2f} MB/s")

    return frame_data


def analyze_frame(data):
    """Analyze frame data for patterns."""
    if len(data) < FRAME_BYTES:
        print(f"WARNING: Incomplete frame: {len(data)}/{FRAME_BYTES} bytes ({100*len(data)/FRAME_BYTES:.1f}%)")

    # Convert to 16-bit values
    import array
    pixels = array.array('H')
    pixels.frombytes(data[:len(data) - len(data) % 2])

    # Buffer is initialized to 0xFFFF, so "empty" pixels are 65535
    EMPTY_VALUE = 65535

    valid_count = sum(1 for p in pixels if p != EMPTY_VALUE)
    empty_count = len(pixels) - valid_count

    print(f"Total pixels: {len(pixels)}")
    print(f"Valid pixels (not 0xFFFF): {valid_count} ({100*valid_count/len(pixels):.1f}%)")
    print(f"Empty pixels (0xFFFF): {empty_count} ({100*empty_count/len(pixels):.1f}%)")

    if len(pixels) > 0:
        min_val = min(pixels)
        max_val = max(pixels)
        # Find min/max excluding empty value
        valid_pixels = [p for p in pixels if p != EMPTY_VALUE]
        if valid_pixels:
            valid_min = min(valid_pixels)
            valid_max = max(valid_pixels)
            print(f"Value range (all): {min_val} - {max_val}")
            print(f"Value range (valid only): {valid_min} - {valid_max}")

    # Check pattern per row - how many valid pixels per row?
    print(f"\nPer-row analysis (first 10 rows):")
    for row in range(min(10, FRAME_HEIGHT)):
        row_start = row * FRAME_WIDTH
        row_end = row_start + FRAME_WIDTH
        row_pixels = pixels[row_start:row_end]
        row_valid = sum(1 for p in row_pixels if p != EMPTY_VALUE)
        print(f"  Row {row}: {row_valid}/{FRAME_WIDTH} valid pixels ({100*row_valid/FRAME_WIDTH:.1f}%)")

    # Find where empty region starts
    empty_start = None
    consecutive_empty = 0
    for i, p in enumerate(pixels):
        if p == EMPTY_VALUE:
            consecutive_empty += 1
            if consecutive_empty > 100 and empty_start is None:
                empty_start = i - 100
        else:
            consecutive_empty = 0

    if empty_start is not None:
        row = empty_start // FRAME_WIDTH
        col = empty_start % FRAME_WIDTH
        print(f"\nEmpty region starts around pixel {empty_start} (row {row}, col {col})")
        print(f"  = {100*empty_start/len(pixels):.1f}% through the frame")

    # Check if it's column-based (every other column empty?)
    print(f"\nColumn pattern check (row 5):")
    row5 = pixels[5*FRAME_WIDTH:6*FRAME_WIDTH]
    even_valid = sum(1 for i, p in enumerate(row5) if i % 2 == 0 and p != EMPTY_VALUE)
    odd_valid = sum(1 for i, p in enumerate(row5) if i % 2 == 1 and p != EMPTY_VALUE)
    print(f"  Even columns valid: {even_valid}/{FRAME_WIDTH//2}")
    print(f"  Odd columns valid: {odd_valid}/{FRAME_WIDTH//2}")

    # Find first and last valid column in row 5
    first_valid = next((i for i, p in enumerate(row5) if p != EMPTY_VALUE), None)
    last_valid = next((i for i, p in enumerate(reversed(row5)) if p != EMPTY_VALUE), None)
    if last_valid is not None:
        last_valid = FRAME_WIDTH - 1 - last_valid
    print(f"  First valid column: {first_valid}")
    print(f"  Last valid column: {last_valid}")

    # Check consecutive valid pixels
    print(f"\nFirst 20 pixels of row 5:")
    for i in range(20):
        val = row5[i]
        status = "VALID" if val != EMPTY_VALUE else "empty"
        print(f"  [{i}] = {val} ({status})")


def main():
    print("=" * 60)
    print("Hamamatsu C7921CA-02 Frame Acquisition Test")
    print("=" * 60)
    print(f"Expected frame: {FRAME_WIDTH}x{FRAME_HEIGHT} = {FRAME_WIDTH*FRAME_HEIGHT} pixels")
    print(f"Expected bytes: {FRAME_BYTES}")
    print()

    handle = connect()

    if not ping(handle):
        return

    get_state(handle)

    if not start_trigger(handle):
        return

    # Wait for acquisition to complete
    print("Waiting for frame acquisition...")
    for _ in range(100):
        time.sleep(0.1)
        state, row, col = get_state(handle)
        if state == 3:  # STATE_DONE
            break
    else:
        print("ERROR: Acquisition did not complete")
        return

    print()
    print("Reading frame data...")
    frame_data = get_frame(handle)

    if frame_data:
        print()
        print("Frame analysis:")
        analyze_frame(frame_data)


if __name__ == "__main__":
    main()
