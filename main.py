#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

import drive
import wander
import fire_detection as fire
import follow_wall as wall

brick = EV3Brick()

# Initialize wheel motors
left = Motor(Port.B)
right = Motor(Port.C)


# ==========================================
# MAIN BEHAVIOR COORDINATION
# ==========================================

def main():

    brick.screen.clear()
    brick.screen.print("Fire Defender")
    brick.screen.print("Starting...")

    wait(1000)

    loop_count = 0

    try:
        while True:

            loop_count += 1

            brick.screen.clear()
            brick.screen.print("Fire Defender")
            brick.screen.print("Loop:", loop_count)

            fire_detected = fire.detect_fire()

            brick.screen.print("Fire:", fire_detected)

            # Priority 1: Fire detection
            if fire_detected:

                brick.screen.print("FIRE DETECTED!")

                drive.stop(left, right)
                fire.alarm()

                brick.screen.print("Extinguishing...")
                fire.extinguish(left, right)

                break

            # Priority 2: Wall following
            elif wall.detect_wall():

                brick.screen.print("Following wall")

                wall.follow_wall(left, right)

                brick.screen.print("Wall step done")

            # Priority 3: Wandering
            else:

                brick.screen.print("Wandering...")

                wander.wander(left, right)

                brick.screen.print("Wander step done")

            wait(50)

    except Exception as error:

        brick.screen.clear()
        brick.screen.print("ERROR:")
        brick.screen.print(str(error))

        wait(5000)
        raise

    finally:

        drive.stop(left, right)
        fire.stop_fan()

        brick.screen.print("Motors stopped")

    brick.screen.clear()
    brick.screen.print("Mission complete!")

    wait(5000)


if __name__ == "__main__":
    main()