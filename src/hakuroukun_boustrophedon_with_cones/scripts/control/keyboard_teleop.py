#!/usr/bin/env python3
"""
keyboard_teleop.py
Keyboard manual control for Hakuroukun robot.
Sends serial commands to keyboardmode.ino running on /dev/arduino.

Controls:
  W        - Accelerate (in current gear)
  B        - Release pedal / depress throttle
  A        - Steer Left
  D        - Steer Right
  S        - Toggle gear FWD <-> REV (interlock enforced in firmware)
  SPACE    - Stop all motors
  Q / ESC / Ctrl+C - Quit
"""

import sys
import tty
import termios
import select
import serial
import time
import argparse

DEFAULT_PORT = '/dev/arduino'
BAUD_RATE    = 9600
KEY_TIMEOUT  = 0.1

KEY_MAP = {
    'w': 'w',   # accelerate
    'b': 'b',   # release pedal
    'a': 'a',   # steer left
    'd': 'd',   # steer right
    's': 's',   # toggle gear
    ' ': ' ',   # stop all
}
QUIT_KEYS = {'\x1b', 'q', 'Q', '\x03'}

KEY_LABELS = {
    'w': '[W] ACCEL   >>',
    'b': '[B] RELEASE --',
    'a': '[A] LEFT     <',
    'd': '[D] RIGHT    >',
    's': '[S] GEAR    **',
    ' ': '[-] STOP    --',
}


def get_key(timeout=KEY_TIMEOUT):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


def drain_serial(ser):
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
    print("=" * 50)
    print("  Hakuroukun Keyboard Teleop")
    print("=" * 50)
    print("  W = Accelerate    B = Release pedal")
    print("  A = Steer Left    D = Steer Right")
    print("  S = Toggle gear   SPACE = Stop all")
    print("  Q / ESC / Ctrl+C  = Quit")
    print("=" * 50)


def main():
    parser = argparse.ArgumentParser(description='Hakuroukun keyboard teleop')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    args = parser.parse_args()

    print_banner()
    print("Connecting to Arduino...")

    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.05)
    except serial.SerialException as e:
        print(f"\n[ERROR] Cannot open {args.port}: {e}")
        print(f"  Try: python3 keyboard_teleop.py --port /dev/ttyACM0")
        sys.exit(1)

    time.sleep(2)
    drain_serial(ser)
    print(f"Connected to {args.port} at {BAUD_RATE} baud.\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    pm_line = ""

    try:
        tty.setraw(fd)

        while True:
            key = get_key(timeout=KEY_TIMEOUT)

            if key in QUIT_KEYS:
                ser.write(b' ')
                break

            cmd = ' ' if key is None else KEY_MAP.get(key.lower(), ' ')
            ser.write(cmd.encode())

            latest = drain_serial(ser)
            if latest:
                pm_line = latest

            label = KEY_LABELS.get(cmd, '[-] STOP    --')
            status = f"{label}  |  {pm_line}"

            # Clear the current line and reprint status — cursor stays on same line
            # \r moves to start of line, \033[K clears from cursor to end of line
            sys.stdout.write(f"\r\033[K{status}")
            sys.stdout.flush()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        try:
            ser.write(b' ')
            ser.close()
        except Exception:
            pass
        print("\n\nMotors stopped. Bye!")


if __name__ == '__main__':
    main()