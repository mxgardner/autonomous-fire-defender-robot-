
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

ev3 = EV3Brick()

# Third motor connected to Port A
fan = Motor(Port.A)

print("Starting fan motor")

# Run slowly for 2 seconds
fan.run(150)
wait(2000)

# Stop the motor
fan.stop()

print("Fan motor stopped")
