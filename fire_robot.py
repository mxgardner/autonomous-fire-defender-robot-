#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait

ev3 = EV3Brick()

# Wheel motors
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Fan motor (same port used in successful fan test)
fan_motor = Motor(Port.A)

# Color sensor
color_sensor = ColorSensor(Port.S3)

# Movement speed
DRIVE_SPEED = 50

print("Fire-fighting robot started!")

# Start moving forward
left_motor.run(DRIVE_SPEED)
right_motor.run(DRIVE_SPEED)

while True:

    detected_color = color_sensor.color()

    if detected_color == Color.RED:

        print("FIRE DETECTED!")

        # Stop both wheels
        left_motor.brake()
        right_motor.brake()

        ev3.speaker.beep()

        # Activate fan
        fan_motor.run(1000)
        print("Fan ON")

        # Run fan for 5 seconds
        wait(5000)

        # Stop fan
        fan_motor.stop()
        print("Fan OFF")

        # End program after detecting fire
        break

    wait(50)

# Make sure all motors are stopped
left_motor.brake()
right_motor.brake()
fan_motor.stop()

print("Mission complete!")
