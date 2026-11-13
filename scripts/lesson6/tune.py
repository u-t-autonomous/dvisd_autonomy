
import curses
import json
import traceback
from dataclasses import dataclass, asdict
from pathlib import Path

from dvisd_autonomy.control.control import Control


CONFIG_PATH = Path("control_calibration.json")


@dataclass
class Calibration:
    esc_neutral_us: int = 1705
    neutral_angle: int = 100
    steering_min: int = 50
    steering_max: int = 140


PARAMETERS = [
    "esc_neutral_us",
    "neutral_angle",
    "steering_min",
    "steering_max",
]

BOUNDS = {
    "esc_neutral_us": (1400, 2000),
    "neutral_angle": (0, 180),
    "steering_min": (0, 180),
    "steering_max": (0, 180),
}


class ControlTuner:
    def __init__(self):
        self.config = Calibration()
        self.selected = 0
        self.step = 1
        self.status = "Ready"
        self.hardware = None

        if CONFIG_PATH.exists():
            data = json.loads(CONFIG_PATH.read_text())
            self.config = Calibration(**data)

        c = self.config

        # Initialize hardware ONCE.
        self.hardware = Control(
            freq_hz=100,
            esc_neutral_us=c.esc_neutral_us,
            esc_forward_us=c.esc_neutral_us,
            neutral_angle=c.neutral_angle,
            steering_min=c.steering_min,
            steering_max=c.steering_max,
        )

    def apply(self):
        """
        Update hardware parameters without reinitializing I2C/PCA9685.
        """
        c = self.config
        hw = self.hardware

        # Update configuration fields directly.
        hw.esc_neutral_us = c.esc_neutral_us
        hw.neutral_angle = c.neutral_angle
        hw.steering_min = c.steering_min
        hw.steering_max = c.steering_max

        # Apply ESC neutral pulse immediately.
        hw.stop()

        # Move servo to the selected calibration point.
        self.move_servo()

    def move_servo(self):
        """Move servo to selected parameter's angle."""
        name = PARAMETERS[self.selected]

        if name == "esc_neutral_us":
            angle = self.config.neutral_angle
        else:
            angle = getattr(self.config, name)

        self.hardware.turn(angle)

    def adjust(self, direction):
        name = PARAMETERS[self.selected]
        old_value = getattr(self.config, name)

        low, high = BOUNDS[name]
        new_value = max(
            low,
            min(high, old_value + direction * self.step)
        )

        # Maintain valid steering limits.
        if name == "steering_min":
            new_value = min(new_value, self.config.steering_max)
        elif name == "steering_max":
            new_value = max(new_value, self.config.steering_min)

        if new_value == old_value:
            return

        setattr(self.config, name, new_value)

        # Immediate hardware update; no reinitialization.
        self.apply()

        self.status = f"Updated {name}: {new_value}"

    def save(self):
        CONFIG_PATH.write_text(
            json.dumps(asdict(self.config), indent=4)
        )
        self.status = f"Saved to {CONFIG_PATH}"

    def draw(self, screen):
        screen.erase()

        lines = [
            "=== RC CONTROL CALIBRATION ===",
            "",
            "UP / DOWN     Select parameter",
            "LEFT / RIGHT  Adjust value",
            "[ / ]         Change step",
            "S             Save configuration",
            "Q             Quit",
            "",
            f"Adjustment step: {self.step}",
            "",
        ]

        for i, name in enumerate(PARAMETERS):
            value = getattr(self.config, name)
            unit = "us" if i == 0 else "deg"
            prefix = ">" if self.selected == i else " "

            lines.append(
                f"{prefix} {name:<20} {value:>5} {unit}"
            )

        lines.extend([
            "",
            f"Status: {self.status}",
            "",
            "WARNING: Raise drive wheels during ESC tuning.",
        ])

        height, width = screen.getmaxyx()

        for row, line in enumerate(lines):
            if row >= height:
                break

            screen.addnstr(row, 0, line, max(0, width - 1))

        screen.refresh()

    def run(self, screen):
        curses.curs_set(0)
        screen.keypad(True)
        screen.timeout(-1)

        try:
            while True:
                self.draw(screen)
                key = screen.getch()

                if key == curses.KEY_UP:
                    self.selected = (
                        self.selected - 1
                    ) % len(PARAMETERS)
                    self.move_servo()

                elif key == curses.KEY_DOWN:
                    self.selected = (
                        self.selected + 1
                    ) % len(PARAMETERS)
                    self.move_servo()

                elif key == curses.KEY_LEFT:
                    self.adjust(-1)

                elif key == curses.KEY_RIGHT:
                    self.adjust(1)

                elif key == ord("["):
                    self.step = max(1, self.step // 2)

                elif key == ord("]"):
                    self.step = min(100, self.step * 2)

                elif key in (ord("s"), ord("S")):
                    self.save()

                elif key in (ord("q"), ord("Q")):
                    break

        finally:
            if self.hardware is not None:
                self.hardware.shutdown()


if __name__ == "__main__":
    try:
        curses.wrapper(
            lambda screen: ControlTuner().run(screen)
        )
    except KeyboardInterrupt:
        print("Interrupted. Hardware shutdown requested.")
    except Exception:
        traceback.print_exc()
        raise

