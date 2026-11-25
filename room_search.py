#!/usr/bin/env pybricks-micropython

from math import atan2, cos, degrees, pi, radians, sin, sqrt

import config as cfg
import drive
from follow_wall import front_pressed

PROBE_DISTANCE = 3500  # mm
MIN_PROBE_TRAVEL = 5  # mm required to treat a stall as a wall measurement
TURN_CLEARANCE = 155  # mm to back away from contact before turning
LANE_MARGIN = 130  # mm inward from a measured limit
LANE_SPACING = 130  # mm between the finest predicted sweep lanes


class RoomSearch:
    """Estimate room axes and sweep using only bumper, motor, and gyro readings."""

    def __init__(self, left, right, stop_when):
        self.left = left
        self.right = right
        self.stop_when = stop_when
        self.start_heading = drive.gyro.angle()
        self.x = 0
        self.y = 0
        self.axis = 0

    def heading(self):
        return self.start_heading - drive.gyro.angle()

    def wheel_distance(self):
        wheel_degrees = (self.left.angle() + self.right.angle()) / 2
        return wheel_degrees * pi * cfg.WHEEL_DIAMETER / 360

    def turn_to(self, heading):
        angle = (heading - self.heading() + 180) % 360 - 180
        if self.stop_when() or abs(angle) <= 0.5:
            return True

        try:
            drive.turn(self.left, self.right, angle, stop_when=self.stop_when, tolerance=0.5)
        except drive.MotionStalled as error:
            # Rewind the rotation so the previous straight path can still be retraced.
            try:
                drive.turn(self.left, self.right, -error.turned,
                           stop_when=self.stop_when, tolerance=0.5)
            except drive.MotionStalled:
                pass
            return False
        return True

    def advance(self, distance, guard=False, require_start=False):
        if self.stop_when() or abs(distance) < 2:
            return 0

        start_distance = self.wheel_distance()
        start_heading = self.heading()

        def stop_at_contact():
            return self.stop_when() or front_pressed()

        try:
            drive.straight(self.left, self.right, distance,
                           stop_when=stop_at_contact if guard else self.stop_when)
        except drive.MotionStalled:
            # A chassis corner can hit before the front bumper is pressed.
            progress = abs(self.wheel_distance() - start_distance)
            if distance < 0 or (require_start and progress < MIN_PROBE_TRAVEL):
                raise

        traveled = self.wheel_distance() - start_distance
        heading = radians((start_heading + self.heading()) / 2)
        self.x += traveled * cos(heading)
        self.y += traveled * sin(heading)
        return traveled

    def room_coordinates(self):
        angle = radians(self.axis)
        return (self.x * cos(angle) + self.y * sin(angle),
                self.y * cos(angle) - self.x * sin(angle))

    def seek(self, a, b):
        angle = radians(self.axis)
        dx = a * cos(angle) - b * sin(angle) - self.x
        dy = a * sin(angle) + b * cos(angle) - self.y
        distance = sqrt(dx * dx + dy * dy)
        if distance < 3:
            return True
        if not self.turn_to(degrees(atan2(dy, dx))):
            return False

        traveled = self.advance(distance, guard=True)
        if traveled < distance - 6:
            self.advance(-min(TURN_CLEARANCE, max(0, traveled)))
            return False
        return True

    def learn_axis(self):
        """Estimate wall direction from two parallel approaches to contact."""
        traveled = self.advance(PROBE_DISTANCE, guard=True, require_start=True)
        if self.stop_when() or traveled < 10 or traveled >= PROBE_DISTANCE - 6:
            return False

        contact_x, contact_y = self.x, self.y
        heading = self.heading()
        self.advance(-min(420, traveled))

        for side in (90, -90):
            if not self.turn_to(heading + side):
                continue
            offset = self.advance(220, guard=True)
            if offset < 100 or not self.turn_to(heading):
                self.advance(-offset)
                continue

            probe = self.advance(1500, guard=True)
            dx, dy = self.x - contact_x, self.y - contact_y
            self.advance(-max(0, probe))
            if 10 <= probe < 1490 and dx * dx + dy * dy > 100 ** 2:
                self.axis = degrees(atan2(dy, dx))
                return True

            if self.turn_to(heading - side):
                self.advance(offset, guard=True)
        return False

    def measure_limits(self):
        limits = []
        for axis_index in range(2):
            end_points = []
            for direction in (180, 0):
                if not self.turn_to(self.axis + axis_index * 90 + direction):
                    return None
                traveled = self.advance(PROBE_DISTANCE, guard=True)
                end_points.append(self.room_coordinates()[axis_index])
                self.advance(-max(0, traveled))
            limits.append(sorted(end_points))
        return limits

    def scan_lane(self, axis_index, ordinate, limits):
        low, high = limits
        if high - low < 2 * TURN_CLEARANCE:
            return

        position = self.room_coordinates()[axis_index]
        along = min(max(position, low + TURN_CLEARANCE), high - TURN_CLEARANCE)
        if axis_index == 0:
            arrived = self.seek(along, ordinate)
        else:
            arrived = self.seek(ordinate, along)
        if not arrived:
            return

        directions = (180, 0) if along - low < high - along else (0, 180)
        for direction in directions:
            if self.turn_to(self.axis + axis_index * 90 + direction):
                traveled = self.advance(PROBE_DISTANCE, guard=True)
                self.advance(-min(TURN_CLEARANCE, max(0, traveled)))

    def explore(self):
        if not self.learn_axis():
            return
        limits = self.measure_limits()
        if limits is None or self.stop_when():
            return

        # Sample center and quarter lanes first, then fill gaps on each axis.
        offsets = [0]
        width = max(high - low for low, high in limits)
        divisions = 4
        while True:
            for index in range(1, divisions // 2, 2):
                offset = index / divisions
                offsets.extend((offset, -offset))
            if width / divisions <= LANE_SPACING:
                break
            divisions *= 2

        for offset in offsets:
            for axis_index in range(2):
                if self.stop_when():
                    return
                low, high = limits[1 - axis_index]
                ordinate = (low + high) / 2 + (high - low) * offset
                if low + LANE_MARGIN <= ordinate <= high - LANE_MARGIN:
                    self.scan_lane(axis_index, ordinate, limits[axis_index])
