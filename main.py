#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.tools import StopWatch, wait

import config as cfg
import drive
import fire_detection as fire
import follow_wall as wall
from room_search import RoomSearch
import wander

brick = EV3Brick()
left = Motor(cfg.LEFT_PORT, positive_direction=cfg.LEFT_DIRECTION)
right = Motor(cfg.RIGHT_PORT, positive_direction=cfg.RIGHT_DIRECTION)

RECOVERY_MOVES = ((75, 45), (150, -60), (250, 85))  # mm back, degrees to turn
WALL_STEPS_BEFORE_DEPARTURE = 10


def recover_from_stall(stall, stop_when):
    """Undo a partial turn, back away, and try a different heading."""
    wall.reset_tracking()
    escape_heading = drive.gyro.angle() + stall.turned

    for backoff, angle in RECOVERY_MOVES:
        if stop_when():
            return True
        if stall.distance is not None and stall.distance < 0:
            return False

        try:
            # A small alignment error can wedge the robot again while backing out.
            alignment = drive.gyro.angle() - escape_heading
            drive.turn(left, right, alignment, stop_when=stop_when, tolerance=1)
            drive.straight(left, right, -backoff, stop_when=stop_when)
        except drive.MotionStalled:
            return False

        try:
            drive.turn(left, right, angle, stop_when=stop_when)
            return True
        except drive.MotionStalled as turn_stall:
            stall = turn_stall

    return False


def search_for_target():
    target = fire.TargetMonitor()
    search_timer = StopWatch()

    def should_stop():
        return target.sample() or search_timer.time() >= cfg.SEARCH_TIMEOUT_MS

    def stop_wandering():
        return should_stop() or wall.detect_wall()

    brick.screen.print("Surveying room")
    try:
        RoomSearch(left, right, should_stop).explore()
    except drive.MotionStalled as stall:
        if not recover_from_stall(stall, should_stop):
            return "Path blocked"

    loop_count = 0
    wall_steps = 0
    while not should_stop():
        loop_count += 1
        brick.screen.clear()
        brick.screen.print("Fire Defender")
        brick.screen.print("Loop:", loop_count)

        try:
            if wall.detect_wall():
                brick.screen.print("Following wall")
                if (wall_steps >= WALL_STEPS_BEFORE_DEPARTURE
                        and wall.leave_wall(left, right, should_stop)):
                    wall_steps = 0
                else:
                    wall.follow_wall(left, right, stop_when=should_stop)
                    wall_steps += 1
            else:
                wall_steps = 0
                brick.screen.print("Wandering...")
                wander.wander(left, right, stop_when=stop_wandering)
        except drive.MotionStalled as stall:
            wall_steps = 0
            brick.screen.print("Recovering from stall")
            if not recover_from_stall(stall, should_stop):
                return "Path blocked"

        wait(50)

    if target.sample():
        drive.stop(left, right)
        brick.screen.print("FIRE DETECTED!")
        fire.alarm()
        brick.screen.print("Extinguishing...")
        fire.extinguish(left, right)
        return "Mission complete!"

    return "Search timed out"


def main():
    brick.screen.clear()
    brick.screen.print("Fire Defender")
    brick.screen.print("Starting...")

    try:
        wait(1000)
        result = search_for_target()
    except Exception as error:
        brick.screen.clear()
        brick.screen.print("ERROR:")
        brick.screen.print(str(error))
        raise
    finally:
        drive.stop(left, right)
        fire.stop_fan()

    brick.screen.clear()
    brick.screen.print(result)


if __name__ == "__main__":
    main()
