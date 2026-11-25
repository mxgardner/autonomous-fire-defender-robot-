#!/usr/bin/env python3
"""Exercise the real controller on a deterministic, approximate EV3 model."""

import argparse
from collections import Counter
from contextlib import redirect_stdout
from dataclasses import dataclass
import importlib
import io
from math import cos, degrees, pi, radians, sin
from pathlib import Path
from random import Random
import sys
from types import ModuleType, SimpleNamespace


SEARCH_LIMIT_MS = 180_000
SIMULATION_LIMIT_MS = 190_000
MM_PER_MOTOR_DEGREE = pi * 81.6 / 360
AXLE_TRACK = 133.8
HALF_LENGTH = 100.0
HALF_WIDTH = 80.5
COLOR_SENSOR = (70, 0)
SONAR_MIN_MM = 30
SONAR_MAX_MM = 2500
SONAR_NO_ECHO_MM = 2550
SONAR_BEAM = tuple(radians(angle) for angle in (-10, 0, 10))
CONTROLLER_MODULES = (
    "main", "config", "drive", "fire_detection", "follow_wall", "room_search", "wander"
)
TERMINAL_MESSAGES = ("Mission complete!", "Search timed out", "Path blocked")
SENSOR_LAYOUTS = (
    ("Standard mount", ((115, -62), (115, 62)), (100, 0)),
    ("Extended mount", ((125, -90), (125, 90)), (95, 0)),
)


@dataclass(frozen=True)
class Box:
    x0: float
    y0: float
    x1: float
    y1: float

    def contains(self, x, y):
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1

    def first_intersection(self, start, end):
        # Clip the segment to the wall's interval on each axis.
        minimum, maximum = 0.0, 1.0
        for origin, stop, lower, upper in zip(
            start, end, (self.x0, self.y0), (self.x1, self.y1)
        ):
            delta = stop - origin
            if abs(delta) <= 1e-9:
                if not lower <= origin <= upper:
                    return None
                continue
            first, second = sorted(((lower - origin) / delta, (upper - origin) / delta))
            minimum, maximum = max(minimum, first), min(maximum, second)
            if minimum > maximum:
                return None
        return minimum

    def overlaps_chassis(self, x, y, angle):
        # Compare projections on the wall axes and the two rotated chassis axes.
        cosine, sine = cos(angle), sin(angle)
        abs_cosine, abs_sine = abs(cosine), abs(sine)
        half_width = (self.x1 - self.x0) / 2
        half_height = (self.y1 - self.y0) / 2
        dx, dy = self.x0 + half_width - x, self.y0 + half_height - y
        return (
            abs(dx) < half_width + HALF_LENGTH * abs_cosine + HALF_WIDTH * abs_sine
            and abs(dy) < half_height + HALF_LENGTH * abs_sine + HALF_WIDTH * abs_cosine
            and abs(dx * cosine + dy * sine)
            < HALF_LENGTH + half_width * abs_cosine + half_height * abs_sine
            and abs(-dx * sine + dy * cosine)
            < HALF_WIDTH + half_width * abs_sine + half_height * abs_cosine
        )


def boundary(width, height):
    return (
        Box(-40, -40, 0, height + 40),
        Box(width, -40, width + 40, height + 40),
        Box(0, -40, width, 0),
        Box(0, height, width, height + 40),
    )


def two_rooms(door_bottom, door_top):
    return boundary(3200, 2100) + (
        Box(1590, 0, 1630, door_bottom),
        Box(1590, door_top, 1630, 2100),
    )


@dataclass(frozen=True)
class Course:
    name: str
    walls: tuple
    marker: Box
    starts: tuple


ORIGINAL_COURSES = (
    Course("One room", boundary(2400, 2100), Box(1300, 945, 1452.5, 1250),
           ((500, 450, 0), (450, 1250, 30), (700, 1600, -25), (500, 780, -45))),
    Course("Two rooms", two_rooms(810, 1170), Box(2540, 990, 2692.5, 1295),
           ((500, 450, 0), (450, 1080, 0), (700, 1600, -25), (500, 780, -45))),
)

HOLDOUT_COURSES = (
    Course("One room with moved marker", boundary(2400, 2100), Box(1720, 460, 1872.5, 765),
           ((360, 830, 17), (550, 1790, 8), (900, 1150, -77), (420, 730, 93))),
    Course("Two rooms with moved doorway", two_rooms(1400, 1760), Box(2580, 1260, 2732.5, 1565),
           ((560, 970, 12), (540, 1470, 41), (760, 1690, -12), (390, 660, -71))),
)


class SimulationLimit(BaseException):
    pass


class World:
    def __init__(self, course, pose, front_bumper, sonar_mount):
        self.walls, self.marker = course.walls, course.marker
        self.x, self.y, direction = pose
        self.angle = radians(direction)
        self.front_bumper = front_bumper
        self.sonar_mount = sonar_mount
        self.motors = {}
        self.time = 0
        self.first_color_read = None
        self.alarms = []
        self.fan_started = None
        self.fan_runs = []
        self.terminal = None
        self.marker_crossed = False
        self.furthest_x = self.x
        self.blocked_steps = 0
        self.sonar_reads = 0
        assert not self.collides(self.x, self.y, self.angle), pose

    def collides(self, x, y, angle):
        return any(wall.overlaps_chassis(x, y, angle) for wall in self.walls)

    def point(self, forward, lateral):
        cosine, sine = cos(self.angle), sin(self.angle)
        return (self.x + forward * cosine - lateral * sine,
                self.y + forward * sine + lateral * cosine)

    def touches_marker(self):
        return self.marker.contains(*self.point(*COLOR_SENSOR))

    def front_pressed(self):
        start, end = (self.point(*location) for location in self.front_bumper)
        return any(wall.first_intersection(start, end) is not None for wall in self.walls)

    def sonar_distance(self):
        # Use the nearest in-range return from three forward-facing rays.
        # The model approximates the beam without simulating sound reflections.
        x, y = self.point(*self.sonar_mount)
        start = x, y
        closest = SONAR_NO_ECHO_MM
        for offset in SONAR_BEAM:
            angle = self.angle + offset
            end = x + SONAR_MAX_MM * cos(angle), y + SONAR_MAX_MM * sin(angle)
            nearest = SONAR_NO_ECHO_MM
            for wall in self.walls:
                fraction = wall.first_intersection(start, end)
                if fraction is not None:
                    nearest = min(nearest, fraction * SONAR_MAX_MM)
            if SONAR_MIN_MM <= nearest <= SONAR_MAX_MM:
                closest = min(closest, nearest)
        return round(closest)

    def record_pose(self):
        self.furthest_x = max(self.furthest_x, self.x)
        self.marker_crossed |= self.touches_marker()

    def advance_clock(self, milliseconds):
        self.time += milliseconds
        if self.time >= SIMULATION_LIMIT_MS:
            raise SimulationLimit

    def wait(self, milliseconds):
        left, right = self.motors["B"], self.motors["C"]
        if not left.speed and not right.speed:
            self.record_pose()
            self.advance_clock(milliseconds)
            return

        while milliseconds:
            step = min(milliseconds, 20)
            milliseconds -= step
            self.advance_clock(step)
            left_angle = left.speed * step / 1000
            right_angle = right.speed * step / 1000
            travel = (left_angle + right_angle) / 2 * MM_PER_MOTOR_DEGREE
            rotation = (right_angle - left_angle) * MM_PER_MOTOR_DEGREE / AXLE_TRACK
            middle = self.angle + rotation / 2
            x, y = self.x + travel * cos(middle), self.y + travel * sin(middle)
            angle = self.angle + rotation
            if self.collides(x, y, angle):
                self.blocked_steps += 1
                continue
            self.x, self.y, self.angle = x, y, angle
            left.position += left_angle
            right.position += right_angle
            self.record_pose()

    def install_pybricks(self):
        world = self

        class Motor:
            def __init__(self, port, positive_direction=None):
                self.port, self.position, self.speed = port, 0.0, 0.0
                world.motors[port] = self

            def angle(self):
                return self.position

            def run(self, speed):
                self.speed = speed
                if self.port == "A" and world.fan_started is None:
                    wheels_stopped = not any(world.motors[port].speed for port in ("B", "C"))
                    world.fan_started = (world.time, wheels_stopped)

            def hold(self):
                self.speed = 0
                if self.port == "A" and world.fan_started is not None:
                    started_at, wheels_stopped = world.fan_started
                    world.fan_runs.append((started_at, world.time - started_at, wheels_stopped))
                    world.fan_started = None

            brake = stop = hold

        class TouchSensor:
            def __init__(self, port):
                assert port == "S1", "The front touch sensor is connected to S1"

            def pressed(self):
                return world.front_pressed()

        class UltrasonicSensor:
            def __init__(self, port):
                assert port == "S2", "The forward ultrasonic sensor is connected to S2"

            def distance(self, silent=False):
                world.sonar_reads += 1
                return world.sonar_distance()

        class GyroSensor:
            def __init__(self, port):
                pass

            def angle(self):
                return -degrees(world.angle)

        class ColorSensor:
            def __init__(self, port):
                pass

            def color(self):
                if world.first_color_read is None:
                    world.first_color_read = world.time
                return "red" if world.touches_marker() else "none"

        class StopWatch:
            def __init__(self):
                self.reset()

            def reset(self):
                self.started = world.time

            def time(self):
                return world.time - self.started

        def screen_print(*args):
            if args and args[0] in TERMINAL_MESSAGES:
                world.terminal = args[0]

        def brick():
            return SimpleNamespace(
                screen=SimpleNamespace(clear=lambda: None, print=screen_print),
                speaker=SimpleNamespace(beep=lambda: world.alarms.append(world.time)),
            )

        modules = {
            "pybricks": {},
            "pybricks.hubs": {"EV3Brick": brick},
            "pybricks.ev3devices": dict(Motor=Motor, TouchSensor=TouchSensor,
                                        UltrasonicSensor=UltrasonicSensor, GyroSensor=GyroSensor,
                                        ColorSensor=ColorSensor),
            "pybricks.parameters": dict(
                Port=SimpleNamespace(**{p: p for p in ("A", "B", "C", "S1", "S2", "S3", "S4")}),
                Direction=SimpleNamespace(CLOCKWISE=1, COUNTERCLOCKWISE=-1),
                Color=SimpleNamespace(RED="red"),
            ),
            "pybricks.tools": dict(wait=world.wait, StopWatch=StopWatch),
        }
        for name, attributes in modules.items():
            module = ModuleType(name)
            vars(module).update(attributes)
            sys.modules[name] = module


def load_controller(world):
    for name in CONTROLLER_MODULES:
        sys.modules.pop(name, None)
    world.install_pybricks()
    return importlib.import_module("main")


def check_sonar_wall_following():
    course = ORIGINAL_COURSES[0]
    _, front_bumper, sonar_mount = SENSOR_LAYOUTS[0]
    start_x = 2250
    world = World(course, (start_x, 450, 0), front_bumper, sonar_mount)
    controller = load_controller(world)
    wall = sys.modules["follow_wall"]
    assert not world.front_pressed(), "Forward range check must start clear of the bumper"
    assert wall.front_blocked(), "Forward sonar did not detect the near wall"

    distance = wall.read_right_distance(controller.left, controller.right, lambda: False)
    assert distance is not None and 345 <= distance <= 355, distance
    assert abs(degrees(world.angle)) <= 2, "Right probe did not restore the starting heading"

    wall.follow_wall(controller.left, controller.right)
    assert 70 <= start_x - world.x <= 80, "Forward sonar did not trigger a backoff"
    assert 80 <= degrees(world.angle) <= 95, "Robot did not turn away from the near wall"
    assert all(motor.speed == 0 for motor in world.motors.values())

    blind_zone = World(course, (2290, 450, 0), front_bumper, sonar_mount)
    load_controller(blind_zone)
    wall = sys.modules["follow_wall"]
    assert wall.ultrasonic.distance() == SONAR_NO_ECHO_MM
    assert wall.front_blocked(), "Front bumper did not detect contact inside the sonar's blind zone"
    print("Forward sonar, right range probe, and bumper override passed.\n")


def run_trial(course, seed, front_bumper, sonar_mount):
    pose = course.starts[seed % len(course.starts)]
    world = World(course, pose, front_bumper, sonar_mount)
    controller = load_controller(world)
    sys.modules["wander"].randint = Random(seed).randint
    assert controller.cfg.SEARCH_TIMEOUT_MS == SEARCH_LIMIT_MS, "Expected a 180-second search limit"

    with redirect_stdout(io.StringIO()):
        try:
            controller.main()
        except SimulationLimit:
            world.terminal = "Exceeded simulation limit"

    assert set(world.motors) == {"A", "B", "C"}
    assert all(motor.speed == 0 for motor in world.motors.values()), "A motor is still running"
    assert world.fan_started is None, "The fan was not stopped"
    if world.terminal == "Mission complete!":
        assert len(world.alarms) == len(world.fan_runs) == 1, "Expected one alarm and fan attempt"
        alert_time = world.alarms[0]
        fan_start, fan_duration, wheels_stopped = world.fan_runs[0]
        assert world.marker_crossed, "Alarm sounded without crossing the marker"
        assert alert_time - world.first_color_read <= SEARCH_LIMIT_MS, "Target found after deadline"
        assert fan_start >= alert_time and fan_duration == 5000 and wheels_stopped
    else:
        assert not world.alarms and not world.fan_runs, "Unexpected fire response"
        assert world.terminal in ("Search timed out", "Path blocked"), world.terminal
    return world


def run_suite(name, courses, trials_per_course):
    outcomes = Counter()
    print(name)
    for sensor_layout, front_bumper, sonar_mount in SENSOR_LAYOUTS:
        for course in courses:
            results = [run_trial(course, seed, front_bumper, sonar_mount)
                       for seed in range(trials_per_course)]
            outcomes.update(result.terminal for result in results)
            completed = [result.time / 1000 for result in results
                         if result.terminal == "Mission complete!"]
            timing = f" ({min(completed):.1f}–{max(completed):.1f} s total)" if completed else ""
            sonar_reads = sum(result.sonar_reads for result in results)
            print(f"  {sensor_layout}, {course.name.lower()} — "
                  f"{len(completed)}/{len(results)} complete{timing}, "
                  f"{sonar_reads} sonar reads")
            for seed, result in enumerate(results):
                if result.terminal != "Mission complete!":
                    print(f"    seed {seed}, start {course.starts[seed % len(course.starts)]} — "
                          f"{result.terminal}, x reached {result.furthest_x:.0f} mm, "
                          f"marker crossed {result.marker_crossed}, "
                          f"blocked updates {result.blocked_steps}")
    completed = outcomes["Mission complete!"]
    total = sum(outcomes.values())
    print(f"  Total — {completed}/{total} complete\n")
    return completed == total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--holdout", action="store_true",
                        help="also report 16 runs with changed starts, marker and doorway")
    args = parser.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    sys.dont_write_bytecode = True
    check_sonar_wall_following()
    passed = run_suite("Original coverage suite", ORIGINAL_COURSES, 20)
    if args.holdout:
        run_suite("Additional coverage scenarios", HOLDOUT_COURSES, 4)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
