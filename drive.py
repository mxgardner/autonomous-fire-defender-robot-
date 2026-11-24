#!/usr/bin/env pybricks-micropython

from math import pi
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
import config as cfg

from pybricks.hubs import EV3Brick

brick = EV3Brick()

gyro = GyroSensor(Port.S4)

PROGRESS_TIMEOUT_MS = 600
MIN_PROGRESS_DEG = 1


class MotionStalled(RuntimeError):
    """A drive command stalled or did not finish before its timeout."""

    def __init__(self, message, distance=None, turned=0):
        super().__init__(message)
        self.distance = distance
        self.turned = turned


# ==========================================
# STRAIGHT MOVEMENT
# ==========================================

def straight(left, right, distance, stop_when=None):
    """Drive straight for a distance in mm, or stop when the callback is true."""

    target = distance * 360 / (pi * cfg.WHEEL_DIAMETER)

    left_start = left.angle()
    right_start = right.angle()
    target_heading = gyro.angle()

    kp = cfg.STRAIGHT_KP
    speed = cfg.SPEED

    timer = StopWatch()
    timeout = 3000 + 3000 * abs(target) / speed
    best_remaining = abs(target)
    last_progress = 0

    try:
        while True:
            if stop_when is not None and stop_when():
                break

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

            elapsed = timer.time()
            if elapsed > timeout:
                raise MotionStalled("Straight movement timed out", distance=distance)

            if best_remaining - abs(remaining) >= MIN_PROGRESS_DEG:
                best_remaining = abs(remaining)
                last_progress = elapsed
            elif elapsed - last_progress >= PROGRESS_TIMEOUT_MS:
                raise MotionStalled("Straight movement stalled", distance=distance)

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

def turn(left, right, angle, stop_when=None, tolerance=5):
    """Turn using gyro feedback, or stop when the callback is true."""

    start = gyro.angle()
    target = start - angle

    kp = 2.0
    max_speed = cfg.SPEED
    min_speed = min(50, max_speed)

    timer = StopWatch()
    display_timer = StopWatch()

    estimated_wheel_angle = (
        cfg.AXLE_TRACK * abs(angle) / cfg.WHEEL_DIAMETER
    )

    timeout = (
        5000 + 9000 * estimated_wheel_angle / max_speed
    )
    best_remaining = abs(angle)
    last_progress = 0

    try:
        while True:
            if stop_when is not None and stop_when():
                break

            current = gyro.angle()
            error = target - current

            # Update robot screen every 250 ms
            if display_timer.time() >= 250:
                brick.screen.clear()
                brick.screen.print("Turning...")
                brick.screen.print("Angle:", angle)
                brick.screen.print("Target:", target)
                brick.screen.print("Current:", current)
                brick.screen.print("Error:", error)

                display_timer.reset()

            # Stop when sufficiently close to target
            if abs(error) <= tolerance:
                break

            elapsed = timer.time()
            if elapsed > timeout:
                raise MotionStalled("Gyro turn timed out", turned=start - current)

            if best_remaining - abs(error) >= MIN_PROGRESS_DEG:
                best_remaining = abs(error)
                last_progress = elapsed
            elif elapsed - last_progress >= PROGRESS_TIMEOUT_MS:
                raise MotionStalled("Gyro turn stalled", turned=start - current)

            # Calculate turning speed
            speed = kp * abs(error)

            speed = max(min_speed, min(max_speed, speed))

            # Determine turning direction
            if error < 0:
                left.run(-speed)
                right.run(speed)
            else:
                left.run(speed)
                right.run(-speed)

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
