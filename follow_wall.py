#!/usr/bin/env pybricks-micropython
# Paul's section

from pybricks.ev3devices import TouchSensor

import config as cfg
import drive

front_bumper = TouchSensor(cfg.FRONT_BUMPER_PORT)
right_bumper = TouchSensor(cfg.RIGHT_BUMPER_PORT)

STEP_DISTANCE = 35  # mm
BACKOFF_DISTANCE = 45  # mm
CORRECTION_ANGLE = 12  # degrees left, to limit the rear corner's swing
CORNER_ANGLE = 90  # degrees left
SEARCH_ANGLE = -25  # degrees right
SEARCH_STEPS = 5
DEPARTURE_ANGLE = 80  # degrees left after one rightward reacquisition step

_search_steps_remaining = 0


def bumper_pressed():
    return front_bumper.pressed() or right_bumper.pressed()


def detect_wall():
    """Return whether a bumper is pressed or wall reacquisition is in progress."""
    return _search_steps_remaining > 0 or bumper_pressed()


def reset_tracking():
    """Clear wall reacquisition after the robot changes course."""
    global _search_steps_remaining
    _search_steps_remaining = 0


def leave_wall(left, right, stop_when):
    """Turn into open space between wall contacts to vary the search path."""
    # First give a doorway or corner one normal wall-reacquisition step.
    if _search_steps_remaining != SEARCH_STEPS - 1 or bumper_pressed():
        return False

    reset_tracking()
    drive.turn(left, right, DEPARTURE_ANGLE,
               stop_when=lambda: stop_when() or bumper_pressed())
    return True


def follow_wall(left, right, stop_when=None):
    """Take one movement step while keeping the wall on the robot's right."""
    global _search_steps_remaining

    def stop_at_front():
        return (stop_when is not None and stop_when()) or front_bumper.pressed()

    def stop_at_bumper():
        return (stop_when is not None and stop_when()) or bumper_pressed()

    if front_bumper.pressed():
        _search_steps_remaining = SEARCH_STEPS
        drive.straight(left, right, -BACKOFF_DISTANCE, stop_when=stop_when)
        drive.turn(left, right, CORNER_ANGLE, stop_when=stop_at_front)
    elif right_bumper.pressed():
        _search_steps_remaining = SEARCH_STEPS
        drive.turn(left, right, CORRECTION_ANGLE, stop_when=stop_at_front)
        drive.straight(left, right, STEP_DISTANCE, stop_when=stop_at_front)
    elif _search_steps_remaining > 0:
        _search_steps_remaining -= 1
        drive.turn(left, right, SEARCH_ANGLE, stop_when=stop_at_bumper)
        drive.straight(left, right, STEP_DISTANCE, stop_when=stop_at_bumper)
