#!/usr/bin/env pybricks-micropython

from random import randint
import drive

# Wandering configuration
STEP_DISTANCE = 100
TURN_CHANCE = 20
MIN_TURN_ANGLE = 30
MAX_TURN_ANGLE = 50


# ==========================================
# WANDERING
# ==========================================

def wander(left, right, stop_when=None):
    """Take one step through open space, stopping early when requested."""

    # Randomly decide whether to turn
    if randint(1, 100) <= TURN_CHANCE:

        # Choose a random turning angle
        angle = randint(MIN_TURN_ANGLE, MAX_TURN_ANGLE)

        # Randomly choose left or right
        if randint(0, 1) == 0:
            angle = -angle

        print("Wandering: Turning", angle, "degrees")

        drive.turn(left, right, angle, stop_when=stop_when)

    else:

        # Continue moving forward
        print("Wandering: Moving forward")

        drive.straight(left, right, STEP_DISTANCE, stop_when=stop_when)