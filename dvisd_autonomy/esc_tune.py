"""
Find the pulse width where the car starts moving. Wheels off the ground.

source ~/myvenv/bin/activate
python -m scripts.lesson5.esc_tune

  +  raise the pulse by 5 us and send it
  -  lower the pulse by 5 us and send it
  space  back to neutral
  q  quit
"""

import sys
import tty
import termios
from dvisd_autonomy.control.control import Control

FREQ = 50
NEUTRAL = 1650
STEP = 5
CENTER_ANGLE = 100
STEER_MIN = 50
STEER_MAX = 140


def get_key():
    """Reads a single keypress from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


car = Control(FREQ, NEUTRAL, NEUTRAL, CENTER_ANGLE, STEER_MIN, STEER_MAX)
pulse = NEUTRAL

print("+ raise 5 us, - lower 5 us, space = neutral, q = quit")
print(f"Now at {pulse} us (neutral).\r")

try:
    while True:
        key = get_key()
        if key in ('+', '='):
            pulse += STEP
        elif key in ('-', '_'):
            pulse -= STEP
        elif key == ' ':
            pulse = NEUTRAL
        elif key in ('q', '\x03'):
            break
        else:
            continue
        car.forward(pulse)
        print(f"Sent {pulse} us\r")
except KeyboardInterrupt:
    pass
finally:
    car.shutdown()
    print("Back to neutral. Cleaned up.")
