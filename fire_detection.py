#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import StopWatch, wait
import drive

brick = EV3Brick()

color_sensor = ColorSensor(Port.S3)
fan_motor = Motor(Port.A)

FAN_SPEED = 1000
FAN_DURATION = 5000
TARGET_COLOR = Color.RED
TARGET_CONFIRMATION_MS = 40


# ==========================================
# FIRE DETECTION
# ==========================================

def detect_fire():
    """Return True if the color sensor sees the configured target color."""

    detected_color = color_sensor.color()

    return detected_color == TARGET_COLOR


class TargetMonitor:
    """Confirm the target color across frequent samples and retain the result for the mission."""

    def __init__(self):
        self._color_timer = StopWatch()
        self._tracking_color = False
        self._confirmed = False

    def sample(self):
        """Return True once the color confirmation interval has elapsed."""
        if self._confirmed:
            return True

        if not detect_fire():
            self._tracking_color = False
        elif self._tracking_color:
            self._confirmed = self._color_timer.time() >= TARGET_CONFIRMATION_MS
        else:
            self._color_timer.reset()
            self._tracking_color = True

        return self._confirmed


# ==========================================
# FIRE ALARM
# ==========================================

def alarm():
    """Play an audible alarm when fire is detected."""

    print("FIRE DETECTED!")

    brick.speaker.beep()


# ==========================================
# FIRE EXTINGUISHING
# ==========================================

def extinguish(left, right):
    """Activate the fan."""

    try:
        # Activate fan
        fan_motor.run(FAN_SPEED)
        print("Fan ON")

        # Run fan for five seconds
        wait(FAN_DURATION)

    finally:
        # Ensure fan stops even if an error occurs
        fan_motor.stop()
        print("Fan OFF")


# ==========================================
# STOP FAN
# ==========================================

def stop_fan():
    """Stop the fan motor."""

    fan_motor.stop()