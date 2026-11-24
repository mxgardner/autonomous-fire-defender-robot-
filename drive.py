#!/usr/bin/env pybricks-micropython

from math import pi
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
import config as cfg

gyro = GyroSensor(Port.S4)


# ==========================================
# STRAIGHT MOVEMENT
# ==========================================

def straight(left, right, distance):
    """Drive straight for a specified distance in mm."""

    target = distance * 360 / (pi * cfg.WHEEL_DIAMETER)

    left_start = left.angle()
    right_start = right.angle()
    target_heading = gyro.angle()

    kp = 0.8
    speed = cfg.SPEED

    timer = StopWatch()
    timeout = 3000 + 3000 * abs(target) / speed

    try:
        while True:
            # Measure distance traveled
            left_moved = left.angle() - left_start
            right_moved = right.angle() - right_start
            traveled = (left_moved + right_moved) / 2

            remaining = target - traveled

            # Calculate heading error
            error = gyro.angle() - target_heading

            # Stop when destination is reached
            if abs(remaining) <= 2:
                break

            # Calculate steering correction
            correction = kp * error

            # Slow down near destination
            base = max(-speed * 0.7,
                       min(speed * 0.7, 4 * remaining))

            # Limit steering correction
            correction = max(-speed / 2,
                             min(speed / 2, correction))

            # Apply corrected motor speeds
            left_speed = base - correction
            right_speed = base + correction

            left.run(max(-speed, min(speed, left_speed)))
            right.run(max(-speed, min(speed, right_speed)))

            if timer.time() > timeout:
                raise RuntimeError("Straight movement timed out")

            wait(20)

    finally:
        left.hold()
        right.hold()


# ==========================================
# INDIVIDUAL WHEEL MOVEMENT
# ==========================================

def move_wheels(left, right, left_angle, right_angle):
    """Rotate each wheel by a specified number of degrees."""

    timer = StopWatch()

    timeout = (
        3000 + 3000 *
        max(abs(left_angle), abs(right_angle)) / cfg.SPEED
    )

    try:
        # Start both motors without blocking
        left.run_angle(cfg.SPEED, left_angle, wait=False)
        right.run_angle(cfg.SPEED, right_angle, wait=False)

        # Wait until both motors finish
        while not (left.control.done() and right.control.done()):

            if timer.time() > timeout:
                raise RuntimeError("Motor movement timed out")

            wait(10)

    finally:
        left.hold()
        right.hold()

    wait(100)


# ==========================================
# GYRO-BASED TURNING
# ==========================================

def turn(left, right, angle):
    """Turn the robot by a specified number of degrees."""

    # Positive angle = counterclockwise (left)
    # Negative angle = clockwise (right)
    target = gyro.angle() - angle

    kp = 1.5
    tolerance = 2
    min_speed = 10
    max_speed = cfg.SPEED

    timer = StopWatch()

    estimated_wheel_angle = (
        cfg.AXLE_TRACK * abs(angle) / cfg.WHEEL_DIAMETER
    )

    timeout = (
        3000 + 6000 * estimated_wheel_angle / max_speed
    )

    settled = 0

    try:
        while True:
            current = gyro.angle()
            error = target - current

            # Require three readings within tolerance
            if abs(error) <= tolerance:
                settled += 1

                left.hold()
                right.hold()

                if settled >= 3:
                    break

            else:
                settled = 0

                # Proportional turning controller
                speed = kp * abs(error)
                speed = max(min_speed, min(max_speed, speed))

                if error < 0:
                    left.run(-speed)
                    right.run(speed)
                else:
                    left.run(speed)
                    right.run(-speed)

            if timer.time() > timeout:
                raise RuntimeError("Gyro turn timed out")

            wait(20)

    finally:
        left.hold()
        right.hold()

    wait(100)


# ==========================================
# STOP MOVEMENT
# ==========================================

def stop(left, right):
    """Stop both wheel motors."""

    left.brake()
    right.brake()