#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import UltrasonicSensor, TouchSensor
from pybricks.parameters import Port

import drive

# Sensor initialization
ultrasonic = UltrasonicSensor(Port.S2)
touch_sensor = TouchSensor(Port.S1)

# Wall following configuration (millimeters)
WALL_DETECTION_DISTANCE = 200
TARGET_WALL_DISTANCE = 150
WALL_TOLERANCE = 30
MAX_WALL_DISTANCE = 300

STEP_DISTANCE = 50
BACKOFF_DISTANCE = 75

# Turning configuration (degrees)
CORRECTION_ANGLE = 15
CORNER_ANGLE = 90

# Wall following state
following_wall = False


# ==========================================
# WALL DETECTION
# ==========================================

def detect_wall():
    """Return True when a wall is detected or being followed."""

    distance = ultrasonic.distance()

    return (
        following_wall
        or touch_sensor.pressed()
        or distance <= WALL_DETECTION_DISTANCE
    )


# ==========================================
# WALL FOLLOWING
# ==========================================

def follow_wall(left, right):
    """Perform one step of right-wall following."""

    global following_wall

    front_distance = ultrasonic.distance()

    # --------------------------------------
    # Handle physical contact
    # --------------------------------------

    if touch_sensor.pressed():

        print("Wall: Contact detected")

        drive.straight(left, right, -BACKOFF_DISTANCE)
        drive.turn(left, right, CORNER_ANGLE)

        following_wall = True
        return

    # --------------------------------------
    # Handle walls ahead and inside corners
    # --------------------------------------

    if front_distance <= WALL_DETECTION_DISTANCE:

        print("Wall: Turning left")

        drive.turn(left, right, CORNER_ANGLE)

        following_wall = True
        return

    # --------------------------------------
    # Measure distance to the right wall
    # --------------------------------------

    drive.turn(left, right, -90)

    right_distance = ultrasonic.distance()

    drive.turn(left, right, 90)

    print("Wall distance:", right_distance)

    # --------------------------------------
    # Wall lost or possible doorway
    # --------------------------------------

    if right_distance > MAX_WALL_DISTANCE:

        print("Wall: Opening detected")

        following_wall = False

        drive.turn(left, right, -45)

        return

    # --------------------------------------
    # Too close to the wall
    # --------------------------------------

    if right_distance < TARGET_WALL_DISTANCE - WALL_TOLERANCE:

        print("Wall: Too close")

        drive.turn(left, right, CORRECTION_ANGLE)

    # --------------------------------------
    # Too far from the wall
    # --------------------------------------

    elif right_distance > TARGET_WALL_DISTANCE + WALL_TOLERANCE:

        print("Wall: Too far")

        drive.turn(left, right, -CORRECTION_ANGLE)

    # --------------------------------------
    # Continue following the wall
    # --------------------------------------

    following_wall = True

    if (
        not touch_sensor.pressed()
        and ultrasonic.distance() > WALL_DETECTION_DISTANCE
    ):
        drive.straight(left, right, STEP_DISTANCE)


# ==========================================
# RESET WALL FOLLOWING
# ==========================================

def reset_tracking():
    """Allow the robot to return to wandering."""

    global following_wall

    following_wall = False
def follow_wall(left, right):
    """Placeholder for future wall-following behavior."""

    pass