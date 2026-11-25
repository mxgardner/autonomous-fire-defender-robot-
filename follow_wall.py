#!/usr/bin/env pybricks-micropython

# Paul's section

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import UltrasonicSensor, TouchSensor
from pybricks.parameters import Port

import drive

brick = EV3Brick()

# Sensors
ultrasonic = UltrasonicSensor(Port.S2)
touch_sensor = TouchSensor(Port.S1)

# Wall configuration (mm)
WALL_DETECTION_DISTANCE = 200
TARGET_WALL_DISTANCE = 150
MAX_WALL_DISTANCE = 350

STEP_DISTANCE = 50
BACKOFF_DISTANCE = 75

# Turning configuration
TURN_ANGLE = 40
CORRECTION_ANGLE = 15

# Wall tracking state
following_wall = False
clear_steps = 0
MAX_CLEAR_STEPS = 3


# ==========================================
# WALL DETECTION
# ==========================================

def detect_wall():
    """Detect nearby walls or continue wall tracking."""

    return (
        following_wall
        or touch_sensor.pressed()
        or ultrasonic.distance() <= WALL_DETECTION_DISTANCE
    )


# ==========================================
# WALL FOLLOWING
# ==========================================

def follow_wall(left, right):
    """Perform one step of wall following."""

    global following_wall
    global clear_steps

    following_wall = True

    distance = ultrasonic.distance()

    brick.screen.clear()
    brick.screen.print("Following wall")
    brick.screen.print("Distance:", distance)
    brick.screen.print("Touch:", touch_sensor.pressed())

    # --------------------------------------
    # Physical contact with wall
    # --------------------------------------

    if touch_sensor.pressed():

        brick.screen.print("Backing away")

        drive.straight(left, right, -BACKOFF_DISTANCE)

        brick.screen.print("Turning left")

        drive.turn(left, right, TURN_ANGLE)

        clear_steps = 0
        return

    # --------------------------------------
    # Wall directly ahead
    # --------------------------------------

    if distance <= WALL_DETECTION_DISTANCE:

        brick.screen.print("Wall ahead")
        brick.screen.print("Turning left")

        drive.turn(left, right, TURN_ANGLE)

        clear_steps = 0
        return

    # --------------------------------------
    # Wall potentially lost
    # --------------------------------------

    elif distance > MAX_WALL_DISTANCE:

        clear_steps += 1

        if clear_steps >= MAX_CLEAR_STEPS:

            brick.screen.print("Wall lost")

            following_wall = False
            clear_steps = 0
            return

        brick.screen.print("Searching right")

        drive.turn(left, right, -CORRECTION_ANGLE)

        return

    # --------------------------------------
    # Follow the wall
    # --------------------------------------

    else:

        clear_steps = 0

        brick.screen.print("Moving forward")

        drive.straight(left, right, STEP_DISTANCE)


# ==========================================
# RESET WALL TRACKING
# ==========================================

def reset_tracking():

    global following_wall
    global clear_steps

    following_wall = False
    clear_steps = 0