# Fire Defender Robot

## Project Overview

The Fire Defender Robot is an autonomous LEGO Mindstorms EV3
robot programmed using Python and Pybricks MicroPython.

The robot navigates its environment through wandering and
wall following, detects a simulated fire target, sounds an
audible alarm, and activates a motor-powered fan to attempt
to extinguish a candle.

## Project Objectives

- Navigate autonomously through an environment.
- Detect and follow walls.
- Navigate corners and avoid getting stuck.
- Wander through open spaces.
- Detect a fire target using a color sensor.
- Sound an audible alarm when a target is detected.
- Activate a fan to extinguish a candle.

## Robot Behavior Diagram

```mermaid
flowchart TD
    A([Start Robot]) --> B[Initialize Motors and Sensors]
    B --> C{Fire Detected?}

    C -->|Yes| D[Stop Wheel Motors]
    D --> E[Sound Alarm]
    E --> F[Activate Fan for 5 Seconds]
    F --> G[Stop All Motors]
    G --> H([Mission Complete])

    C -->|No| I{Wall Detected?}
    I -->|Yes| J[Follow Wall]
    I -->|No| K[Wander]

    J --> C
    K --> C

    classDef startEnd fill:#64748b,color:#fff
    classDef decision fill:#eab308,color:#000
    classDef fire fill:#dc2626,color:#fff
    classDef wall fill:#2563eb,color:#fff
    classDef wander fill:#16a34a,color:#fff

    class A,H startEnd
    class C,I decision
    class D,E,F fire
    class J wall
    class K wander
```

## Project Structure

```text
fire-defender/
├── main.py
├── config.py
├── drive.py
├── wall_following.py
├── fire_detection.py
└── README.md
```

| File | Description |
| --- | --- |
| `main.py` | Wandering and overall behavior coordination |
| `config.py` | Shared robot configuration and constants |
| `drive.py` | Shared movement and turning functions |
| `wall_following.py` | Wall detection and wall following |
| `fire_detection.py` | Fire detection, alarm, and extinguishing |
| `README.md` | Project documentation |

## Team Responsibilities

### Person 1: Robot Construction and Wall Following

**Assigned file:** `wall_following.py`

Responsibilities:

- Build and configure the robot.
- Configure sensors and motors.
- Program wall detection and wall following.
- Handle corners and prevent the robot from getting stuck.

Deliverables:

- `detect_wall()`
- `follow_wall()`
- A working robot capable of navigating along walls.

### Person 2: Wandering and Behavior Coordination

**Assigned file:** `main.py`

Responsibilities:

- Program the robot to move through open spaces.
- Determine when to switch between wandering and wall following.
- Combine all team members' functions into one main program.
- Prioritize fire detection over navigation.

Deliverables:

- `wander()`
- `main()`
- A working program that coordinates all robot behaviors.

### Person 3: Fire Detection, Alarm, and Extinguishing

**Assigned file:** `fire_detection.py`

Responsibilities:

- Program the robot to identify a simulated fire target.
- Program an audible alarm.
- Build and program the fan using a third motor.
- Test whether the fan can extinguish a candle.

Deliverables:

- `detect_fire()`
- `alarm()`
- `extinguish(left, right)`
- `stop_fan()`

## Hardware Configuration

| Component | Port | Purpose |
| --- | --- | --- |
| Left Wheel Motor | B | Robot movement |
| Right Wheel Motor | C | Robot movement |
| Fan Motor | A | Fire extinguishing |
| Color Sensor | S3 | Fire target detection |
| Gyro Sensor | S4 | Heading and turning |
| Wall Detection Sensor | TBD | Wall detection |

## Shared Configuration

The `config.py` file contains shared constants used by the
robot's movement functions.

| Constant | Description |
| --- | --- |
| `SPEED` | Default motor speed |
| `WHEEL_DIAMETER` | Wheel diameter in millimeters |
| `AXLE_TRACK` | Distance between wheels in millimeters |

## Shared Drive Functions

All team members should use `drive.py` for robot movement
instead of implementing separate motor controls.

```python
import drive
```

### Drive Function Reference

| Function | Description |
| --- | --- |
| `straight(left, right, distance)` | Drive a specified distance in millimeters |
| `move_wheels(left, right, left_angle, right_angle)` | Rotate each wheel independently |
| `turn(left, right, angle)` | Turn using gyro feedback |
| `stop(left, right)` | Brake both wheel motors |

### Example Usage

```python
import drive

# Move forward 200 mm
drive.straight(left, right, 200)

# Move backward 100 mm
drive.straight(left, right, -100)

# Turn left 90 degrees
drive.turn(left, right, 90)

# Turn right 90 degrees
drive.turn(left, right, -90)

# Stop the robot
drive.stop(left, right)
```

### Movement Conventions

- Positive distances move the robot forward.
- Negative distances move the robot backward.
- Positive turning angles rotate counterclockwise.
- Negative turning angles rotate clockwise.
- Distances are measured in millimeters.
- Angles are measured in degrees.

## Fire Detection Functions

The `fire_detection.py` module handles target detection,
audible alerts, and fan activation.

```python
import fire_detection as fire
```

| Function | Description |
| --- | --- |
| `detect_fire()` | Returns True when the color sensor detects red |
| `alarm()` | Plays an audible alarm |
| `extinguish(left, right)` | Stops the wheels and activates the fan |
| `stop_fan()` | Stops the fan motor |

### Fan Configuration

| Setting | Value |
| --- | --- |
| Fan Motor Port | A |
| Fan Speed | 1000 degrees per second |
| Fan Duration | 5000 milliseconds |
| Target Color | Red |

### Example Usage

```python
if fire.detect_fire():
    drive.stop(left, right)
    fire.alarm()
    fire.extinguish(left, right)
```

## Main Program

The main program coordinates all robot behaviors.

### Behavior Priority

1. Fire detection and extinguishing
2. Wall following
3. Wandering

Fire detection takes priority over all navigation behaviors.

### Planned Main Program

```python
#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

import drive
import fire_detection as fire
import wall_following as wall

left = Motor(Port.B)
right = Motor(Port.C)


def wander(left, right):
    # TODO: Implement wandering behavior
    drive.straight(left, right, 50)


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
                wander(left, right)

            wait(50)

    finally:
        drive.stop(left, right)
        fire.stop_fan()

    print("Mission complete!")


if __name__ == "__main__":
    main()
```

## Integration Guidelines

- Each team member should work in their assigned file.
- Use `drive.py` for shared movement functions.
- Use `config.py` for shared configuration values.
- Do not initialize the same motor or sensor in multiple files.
- Only `main.py` should contain the main behavior loop.
- Navigation functions should perform short movement steps
  before returning control to the main program.
- Detection functions should return boolean values.
- Motors should stop safely when an error occurs.
- Communicate changes to shared functions before merging.

## Git Collaboration

Each team member should work on their own branch.

| Team Member | Branch |
| --- | --- |
| Person 1 | `wall-following` |
| Person 2 | `wandering-coordination` |
| Person 3 | `fire-detection` |

### Create a Branch

```bash
git switch -c your-branch-name
```

### Save Changes

```bash
git add .
git commit -m "Describe your changes"
git push -u origin your-branch-name
```

### Update Your Local Main Branch

```bash
git switch main
git pull origin main
```

Team members should submit pull requests to merge their
completed work into the main branch.

## Testing Checklist

### Robot Construction and Movement

- [ ] Robot construction completed
- [ ] Motor and sensor ports configured
- [ ] Straight movement tested
- [ ] Gyro-based turning tested
- [ ] Individual wheel movement tested
- [ ] Motor stopping tested

### Wall Following

- [ ] Wall detection implemented
- [ ] Wall following implemented
- [ ] Corner navigation tested
- [ ] Obstacle avoidance tested
- [ ] Robot can recover from difficult positions

### Wandering and Coordination

- [ ] Wandering implemented
- [ ] Wandering-to-wall-following transition tested
- [ ] Wall-following-to-wandering transition tested
- [ ] Main behavior loop implemented
- [ ] Fire detection correctly prioritized

### Fire Detection and Extinguishing

- [ ] Red target detection tested
- [ ] Audible alarm tested
- [ ] Fan motor tested
- [ ] Fan successfully extinguishes a candle
- [ ] All motors stop after an extinguishing attempt

### Final Integration

- [ ] All team branches merged
- [ ] All modules work together
- [ ] Complete autonomous navigation tested
- [ ] Complete fire response tested
- [ ] Final demonstration completed

## Current Limitations

- The color sensor detects red rather than directly detecting
  a flame.
- The robot does not independently verify whether a fire has
  been extinguished.
- Movement functions are blocking, so short movement steps
  are necessary for frequent fire detection.
- Wall detection and wandering interfaces are subject to
  change during development.
- The robot is an educational prototype and is not intended
  for real-world firefighting.

All candle extinguishing tests should be conducted in a
controlled and supervised environment.

## Technologies Used

- LEGO Mindstorms EV3
- Python
- Pybricks MicroPython
- Git
- GitHub
- Mermaid