#!/usr/bin/env pybricks-micropython
# Paul's section

from pybricks.ev3devices import TouchSensor, UltrasonicSensor

import config as cfg
import drive

front_bumper = TouchSensor(cfg.FRONT_BUMPER_PORT)
ultrasonic = UltrasonicSensor(cfg.ULTRASONIC_PORT)

STEP_DISTANCE = 100  # mm between side readings
BACKOFF_DISTANCE = 75  # mm
CORRECTION_ANGLE = 12  # degrees of steering per step
CORNER_ANGLE = 90  # degrees left
SEARCH_ANGLE = -25  # degrees right
SEARCH_STEPS = 5
DEPARTURE_ANGLE = 80  # degrees left after one rightward reacquisition step
SIDE_SCAN_ANGLE = 90  # degrees to the right
SIDE_SCAN_TOLERANCE = 2  # degrees
HEADING_RESTORE_TOLERANCE = 0.5  # degrees to limit drift across repeated scans
WALL_DISTANCE_BAND = 20  # mm on either side of the target distance
WALL_DETECTION_DISTANCE = 260  # mm from the ultrasonic sensor during a side reading

_search_steps_remaining = 0


def front_pressed():
    return front_bumper.pressed()


def front_blocked():
    """Check the bumper or a short forward range reading."""
    return front_pressed() or 0 < ultrasonic.distance() <= cfg.FRONT_STOP_DISTANCE


def detect_wall():
    """Return whether a wall is ahead or wall tracking is in progress."""
    return _search_steps_remaining > 0 or front_blocked()


def reset_tracking():
    """Clear wall reacquisition after the robot changes course."""
    global _search_steps_remaining
    _search_steps_remaining = 0


def leave_wall(left, right, stop_when):
    """Turn into open space after a wall is lost to vary the search path."""
    # First give a doorway or corner one normal wall-reacquisition step.
    if _search_steps_remaining != SEARCH_STEPS - 1 or front_blocked():
        return False

    reset_tracking()
    drive.turn(left, right, DEPARTURE_ANGLE,
               stop_when=lambda: stop_when() or front_pressed())
    return True


def read_right_distance(left, right, stop_when):
    """Point the forward sensor right, measure, and restore the original heading."""
    original_heading = drive.gyro.angle()
    drive.turn(left, right, -SIDE_SCAN_ANGLE,
               stop_when=stop_when, tolerance=SIDE_SCAN_TOLERANCE)
    rotation = drive.gyro.angle() - original_heading
    distance = None
    if abs(rotation - SIDE_SCAN_ANGLE) <= SIDE_SCAN_TOLERANCE and not stop_when():
        distance = ultrasonic.distance()
    drive.turn(left, right, rotation, stop_when=stop_when, tolerance=HEADING_RESTORE_TOLERANCE)
    return distance


def follow_wall(left, right, stop_when=None):
    """Take one movement step while keeping the wall on the robot's right."""
    global _search_steps_remaining

    def stop_at_touch():
        return (stop_when is not None and stop_when()) or front_pressed()

    def stop_at_front():
        return (stop_when is not None and stop_when()) or front_blocked()

    if front_blocked():
        _search_steps_remaining = SEARCH_STEPS
        drive.straight(left, right, -BACKOFF_DISTANCE, stop_when=stop_when)
        drive.turn(left, right, CORNER_ANGLE, stop_when=stop_at_touch)
        return

    if _search_steps_remaining <= 0:
        return

    distance = read_right_distance(left, right, stop_at_touch)
    if distance is not None and 0 < distance <= WALL_DETECTION_DISTANCE:
        _search_steps_remaining = SEARCH_STEPS
        error = distance - cfg.WALL_FOLLOW_DISTANCE
        if error < -WALL_DISTANCE_BAND:
            drive.turn(left, right, CORRECTION_ANGLE, stop_when=stop_at_touch)
        elif error > WALL_DISTANCE_BAND:
            drive.turn(left, right, -CORRECTION_ANGLE, stop_when=stop_at_touch)
    else:
        _search_steps_remaining -= 1
        drive.turn(left, right, SEARCH_ANGLE, stop_when=stop_at_touch)

    drive.straight(left, right, STEP_DISTANCE, stop_when=stop_at_front)
