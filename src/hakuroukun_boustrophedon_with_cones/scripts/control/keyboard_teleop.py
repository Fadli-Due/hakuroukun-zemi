#!/usr/bin/env python3
"""
keyboard_teleop.py
Keyboard manual control for Hakuroukun robot.
Sends serial commands to keyboard_mode.ino running on /dev/arduino.

Controls:
  W        - Accelerate (in current gear)
  B        - Release pedal / depress throttle
  A        - Steer Left
  D        - Steer Right
  S        - Toggle gear FWD <-> REV (interlock enforced in firmware)
  SPACE    - Stop all motors
  Q / ESC / Ctrl+C - Quit

Usage:
  python3 keyboard_teleop.py
  python3 keyboard_teleop.py --port /dev/ttyACM0   # if symlink not set up

Original manualmode.ino by Duc-san (c) 2024 ISE Mobile Robot Group
keyboard_teleop.py by Fadli Due Ramandavito, 2026
"""

import sys
import tty
import termios
import select
import serial
import time
import argparse

# -- Serial port ---------------------------------------------------------------
DEFAULT_PORT = '/dev/arduino'
BAUD_RATE    = 9600

# How long to wait for a keypress before sending auto-stop (seconds)
KEY_TIMEOUT  = 0.1

# -- Key -> command mapping ----------------------------------------------------
KEY_MAP = {
    'w': 'w',   # accelerate (forward or reverse depending on gear)
    'b': 'b',   # release pedal / depress throttle
    'a': 'l',   # steer left
    'd': 'r',   # steer right
    's': 's',   # toggle gear (interlock enforced in firmware)
    ' ': ' ',   # stop all
}
QUIT_KEYS = {'\x1b', 'q', 'Q', '\x03'}  # ESC, q, Q, Ctrl+C

# -- Helpers -------------------------------------------------------------------
def get_key_nonblocking(timeout=KEY_TIMEOUT):
    """
    Wait up to `timeout` seconds for a keypress.
    Returns the character, or None if no key was pressed in time.
    """
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None

def drain_serial(ser):
    """
    Read and discard all bytes currently sitting in the serial buffer.
    Returns the last complete line seen (for PM display), or "".
    Prevents buffer buildup that causes the 15-second freeze.
    """
    last_line = ""
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                last_line = line
        except Exception:
            break
    return last_line

def print_banner():
    print("=" * 45)
    print("  Hakuroukun Keyboard Teleop")
    print("=" * 45)
    print("  W        : Accelerate (in current gear)")
    print("  B        : Release pedal / depress")
    print("  A        : Steer Left")
    print("  D        : Steer Right")
    print("  S        : Toggle gear FWD<->REV (interlock)")
    print("  SPACE    : Stop all motors")
    print("  Q / ESC / Ctrl+C : Quit")
    print("=" * 45)
    print("Connecting to Arduino...")

def print_status(key, cmd, pm_line=""):
    label = {
        'w': 'ACCEL       >>',
        'b': 'PEDAL REL   --',
        'l': 'STEER LEFT   <',
        'r': 'STEER RIGHT  >',
        's': 'GEAR TOGGLE **',
        ' ': 'STOP        --',
    }.get(cmd, 'STOP        --')
    if key is None or key == '':
        key_display = '---'
    elif key == ' ':
        key_display = 'SPC'
    else:
        key_display = key.upper()
    sys.stdout.write(f"\r  Key: [{key_display}]  {label}   {pm_line}        ")
    sys.stdout.flush()

# -- Main ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Hakuroukun keyboard teleop')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    args = parser.parse_args()

    print_banner()

    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.05)
    except serial.SerialException as e:
        print(f"\n[ERROR] Cannot open {args.port}: {e}")
        print("  Try: python3 keyboard_teleop.py --port /dev/ttyACM0")
        sys.exit(1)

    time.sleep(2)   # Wait for Arduino reset after serial open
    drain_serial(ser)  # Discard the startup "keyboard_mode ready." message
    print(f"Connected to {args.port} at {BAUD_RATE} baud.\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    pm_line = ""

    try:
        tty.setraw(fd)

        while True:
            # Non-blocking key read
            key = get_key_nonblocking(timeout=KEY_TIMEOUT)

            # Quit keys
            if key in QUIT_KEYS:
                ser.write(b' ')
                break

            # Resolve command
            if key is None:
                # No key pressed -> auto-stop (space, NOT 's')
                cmd = ' '
                display_key = None
            else:
                cmd = KEY_MAP.get(key.lower(), ' ')
                display_key = key

            # Send to Arduino
            ser.write(cmd.encode())

            # Drain entire serial buffer (fixes the freeze)
            latest = drain_serial(ser)
            if latest:
                pm_line = latest

            print_status(display_key, cmd, pm_line)

    finally:
        # Always restore terminal and stop motors
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        try:
            ser.write(b' ')
            ser.close()
        except Exception:
            pass
        print("\n\nMotors stopped. Bye!")

if __name__ == '__main__':
    main()