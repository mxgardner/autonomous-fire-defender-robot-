#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

import drive
import wander
import fire_detection as fire
import follow_wall as wall

# Initialize wheel motors
left = Motor(Port.B)
right = Motor(Port.C)


# ==========================================
# MAIN BEHAVIOR COORDINATION
# ==========================================

def main():

    print("Fire Defender Robot started!")

    try:
        while True:

            # Priority 1: Fire detection
            if fire.detect_fire():
                drive.stop(left, right)
                fire.alarm()
                fire.extinguish(left, right)
                break

            # Priority 2: Wall following
            elif wall.detect_wall():
                wall.follow_wall(left, right)

            # Priority 3: Wandering
            else:
                wander.wander(left, right)

            wait(50)

    finally:
        drive.stop(left, right)
        fire.stop_fan()

    print("Fire extinguished!")


if __name__ == "__main__":
    main()