from wall_following import follow_wall, detect_wall, setup_robot
from wandering import wander
from fire_detection import detect_fire, alarm, extinguish

import time


def main():
    robot = setup_robot()

    try:
        while True:

            # Priority 1: Detect and extinguish fire
            if detect_fire(robot):
                robot.stop()
                alarm(robot)
                extinguish(robot)
                break

            # Priority 2: Follow walls
            elif detect_wall(robot):
                follow_wall(robot)

            # Priority 3: Wander through open space
            else:
                wander(robot)

            time.sleep(0.05)

    finally:
        robot.stop()


if __name__ == "__main__":
    main()