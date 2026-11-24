from follow_wall
import wander
import drive
import fire_detection as fire

import time


def main():
    robot = setup_robot()

    try:
        while True:

            # Priority 1: Detect and extinguish fire
            if fire.detect_fire():
                drive.stop(left,right)
                fire.alarm()
                fire.extinguish()
                break

            # Priority 2: Follow walls
            elif detect_wall():
                follow_wall()

            # Priority 3: Wander through open space
            else:
                wander.wander(left,right)

            time.sleep(0.05)

    finally:
        robot.stop()


if __name__ == "__main__":
    main()