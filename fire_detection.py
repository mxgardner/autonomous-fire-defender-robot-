
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait

ev3 = EV3Brick()

color_sensor = ColorSensor(Port.S3)

fan_motor = Motor(Port.A)

print("Fire detection system started!")

while True:
    detected_color = color_sensor.color()

    if detected_color == Color.RED:
        print("FIRE DETECTED!")

        ev3.speaker.beep()

        # Turn on fan
        fan_motor.run(500)
        print("Fan ON")

        # Run fan for 5 seconds
        wait(5000)

        # Stop fan
        fan_motor.stop()
        print("Fan OFF")

        # Wait until red is no longer detected
        while color_sensor.color() == Color.RED:
            wait(100)

    wait(100)