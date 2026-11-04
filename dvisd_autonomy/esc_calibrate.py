"""
Guided ESC calibration (neutral, full throttle, full brake). It prints each
step; do what it says, then press Enter. Take the wheels off the ground first.

source ~/myvenv/bin/activate
python -m scripts.lesson5.esc_calibrate
"""

from dvisd_autonomy.control.control import Control

FREQ = 50
NEUTRAL = 1500
FULL_THROTTLE = 2000
FULL_BRAKE = 1000
CENTER_ANGLE = 100
STEER_MIN = 50
STEER_MAX = 140

STEPS = [
    ("NEUTRAL", NEUTRAL),
    ("FULL THROTTLE", FULL_THROTTLE),
    ("FULL BRAKE", FULL_BRAKE),
]

print("Wheels off the ground. Car powered on. Starting (waits ~5 s)...")
car = Control(FREQ, NEUTRAL, NEUTRAL, CENTER_ANGLE, STEER_MIN, STEER_MAX)

try:
    print("\nSending NEUTRAL now.")
    print("STEP 1: Hold the EZ-Set button on the ESC until the LED lights up, then release it.")
    input("Press Enter when the LED is on... ")

    for i, (name, pulse_us) in enumerate(STEPS, start=2):
        car.forward(pulse_us)
        print(f"\nSTEP {i}: Sending {name} ({pulse_us} us).")
        print("Tap EZ-Set ONCE now. The LED should blink to confirm.")
        input("Press Enter after you tapped EZ-Set and saw the blink... ")

    car.stop()
    print("\nDone. Sending neutral. Wait for the LED to settle, then test with:")
    print("  python -m scripts.lesson5.tele_op")
    print("Use FREQ = 50, ESC_NEUTRAL = 1500, ESC_FORWARD ~1550 (raise slowly).")
except KeyboardInterrupt:
    pass
finally:
    car.shutdown()
    print("Back to neutral. Cleaned up.")
