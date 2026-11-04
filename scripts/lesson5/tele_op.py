"""
source ~/myvenv/bin/activate
python -m scripts.lesson5.tele_op
"""

import sys
import tty
import termios
from dvisd_autonomy.control.control import Control
import time

def get_key():
    """Reads a single keypress from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Configuration
FREQ = 50
ESC_NEUTRAL = 1705
ESC_FORWARD = 1810
CENTER_ANGLE = 115
STEER_MIN = 65
STEER_MAX = 165

# Initialize
car = Control(FREQ, ESC_NEUTRAL, ESC_FORWARD, CENTER_ANGLE, STEER_MIN, STEER_MAX)

print("Drive started! Use WASD to move, Q to quit.")
car.stop()
try:
    while True:
        key = get_key().lower()
        print("Entered Key:", key)

        if key == 'w':
            print("Forward")
            car.straight()
            car.forward()
            time.sleep(0.5)
            car.stop()
        elif key == 's':
            print("Stop")
            car.stop()
        elif key == 'a':
            print("Left")
            car.turn(STEER_MIN)
            car.forward()
            time.sleep(0.5)
            car.straight()
            car.stop()
        elif key == 'd':
            print("Right")
            car.turn(STEER_MAX)
            car.forward()
            time.sleep(0.5)
            car.straight()
            car.stop()
        elif key == ' ': # Spacebar to reset
            car.straight()
            car.stop()
        elif key == 'q':
            break
except KeyboardInterrupt:
    pass
finally:
    car.shutdown()
    print("Cleaned up.")