#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
import drive

brick = EV3Brick()

color_sensor = ColorSensor(Port.S3)
fan_motor = Motor(Port.A)

FAN_SPEED = 1000
FAN_DURATION = 5000


# ==========================================
# FIRE DETECTION
# ==========================================

def detect_fire():
    """Return True if the color sensor detects red."""

    detected_color = color_sensor.color()

    return detected_color == Color.RED


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