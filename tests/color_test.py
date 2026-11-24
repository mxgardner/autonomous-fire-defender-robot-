
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait

ev3 = EV3Brick()

# Change S3 if your sensor uses another port
color_sensor = ColorSensor(Port.S3)

while True:
    detected = color_sensor.color()
    print("Detected color:", detected)

    if detected == Color.RED:
        ev3.speaker.beep()
        print("FIRE DETECTED!")

    wait(500)
