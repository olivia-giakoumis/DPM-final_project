"""
ECSE 211 Final Project - Phase 1 (mapping) test, without color sensors.

Sensors: front touch sensor (a press = wall ahead) and a side ultrasonic.

  1. From the origin, take the shortest path through tile centres to the
     approach tile (the tile on the origin side of the s1-s2 edge) and face
     the street. A bump on the way marks a box: back up and go around it.
  2. Walk the street one tile at a time. It is one tile wide with no
     branches, so at each tile centre:
       sensor side wall, no bump  -> straight on
       sensor side wall, bump     -> turn to the other side; bump again = dead end
       sensor side open, bump     -> the street turns toward the sensor side
       sensor side open, no bump  -> we left the street (exit)
     While a box is beside the robot, the ultrasonic keeps it centred and
     corrects its heading.
  3. Retrace the tiles back to the origin and face +y.
  4. Save street_map.json (the format final.py reads).

Usage:               python3 phase1_simple.py 1,3 1,2 2,-1 3,-1   (s1 s2 d1 d2)
Test on a computer:  python3 sim_phase1.py

Coordinates: origin = lower-left intersection. Tile (i, j) is the square whose
lower-left corner is (i, j). Odometry in cm. Heading: 0 = +y, 90 = +x, clockwise.
"""

from utils.brick import (
    Motor, EV3UltrasonicSensor, TouchSensor, reset_brick, wait_ready_sensors,
)
import heapq
import itertools
import json
import math
import re
import sys
import time

# =============================================================================
# ROBOT SETUP - measure these on your robot
# =============================================================================
LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT = "A", "B"
ULTRASONIC_PORT, TOUCH_PORT = 3, 4
MOTOR_DIR = -1             # -1: motors mounted backwards (negative dps = forward)
US_SIDE = -1               # side the ultrasonic faces: -1 = left, +1 = right

# All offsets are from the axle centre (halfway between the wheels), in cm.
RADIUS = 2.13              # wheel radius
TRACK_WIDTH = 14.5         # middle of left tire to middle of right tire
BUMPER_OFFSET = 14.0       # forward, to where the touch sensor clicks
US_SIDE_OFFSET = 8.0       # sideways, to the face of the ultrasonic
US_FORWARD_OFFSET = 0.0    # forward (+) or backward (-), to the ultrasonic

# Speeds (wheel degrees per second)
DRIVE_DPS = 100            # cruise
STOP_DPS = 30              # slowest speed while braking for a stop
PROBE_DPS = 30             # nudging forward to feel for a wall, re-centring
BACKUP_DPS = 90            # reversing
TURN_DPS = 280             # fastest turning speed
TURN_BACKOFF = 2.0         # cm to back away when the front catches a box mid-turn

# The map
TILE = 30.48
MIN_TILE_X, MAX_TILE_X = -1, 7         # intersections x in [-1, 8]
MIN_TILE_Y, MAX_TILE_Y = -1, 3         # intersections y in [-1, 4]
MAP_FILE = "street_map.json"

# From a tile centre a box beside the tile reads ~7 cm and an open side
# reads more than ~38 cm: anything under ~22 cm is a box.
WALL_THRESHOLD = TILE - US_SIDE_OFFSET

DIRS = {0: (0, 1), 90: (1, 0), 180: (0, -1), 270: (-1, 0)}
NAMES = {0: "+y", 90: "+x", 180: "-y", 270: "-x"}

# Results of a move
OK, BUMP, FAIL = "ok", "bump", "fail"
FRONT = "front"    # reached the next tile and bumped the box in front of it

# =============================================================================
# Hardware and state
# =============================================================================
leftmotor = Motor(LEFT_MOTOR_PORT)
rightmotor = Motor(RIGHT_MOTOR_PORT)
us_sensor = EV3UltrasonicSensor(ULTRASONIC_PORT)
touch_sensor = TouchSensor(TOUCH_PORT)
wait_ready_sensors()
leftmotor.reset_encoder()
rightmotor.reset_encoder()

x, y, theta = 0.0, 0.0, 0.0       # odometry (cm, cm, deg)
odo_dist = 0.0                    # total distance driven forward (cm)
prev_left = prev_right = 0
trail = []                        # tiles visited, in order (to retrace home)
wall_run = []                     # (odo_dist, reading, theta) beside a straight wall


# =============================================================================
# Small helpers
# =============================================================================
def set_wheels(left_dps, right_dps):
    leftmotor.set_dps(MOTOR_DIR * left_dps)
    rightmotor.set_dps(MOTOR_DIR * right_dps)


def stop_motors():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_diff(target, current):
    """Signed difference target - current, in [-180, 180)."""
    return (target - current + 180) % 360 - 180


def nearest_axis(heading):
    return round((heading % 360) / 90) % 4 * 90


def update_odometer():
    global x, y, theta, odo_dist, prev_left, prev_right
    left = MOTOR_DIR * leftmotor.get_encoder()
    right = MOTOR_DIR * rightmotor.get_encoder()
    d_left = (left - prev_left) / 360 * 2 * math.pi * RADIUS
    d_right = (right - prev_right) / 360 * 2 * math.pi * RADIUS
    prev_left, prev_right = left, right

    d_theta = (d_left - d_right) / TRACK_WIDTH       # clockwise positive
    d_dist = (d_left + d_right) / 2
    mid = math.radians(theta) + d_theta / 2
    x += d_dist * math.sin(mid)
    y += d_dist * math.cos(mid)
    theta = (theta + math.degrees(d_theta)) % 360
    odo_dist += d_dist


def bumped():
    return bool(touch_sensor.is_pressed())


def read_us():
    """One ultrasonic reading in cm, or None."""
    value = us_sensor.get_value()
    return float(value) if value is not None and value > 0 else None


def read_us_median(samples=7):
    """
    Median of a few ultrasonic readings. 255 means "no echo", which the
    sensor sometimes reports by mistake, so it only counts if every reading
    is 255.
    """
    values = []
    for _ in range(samples):
        value = read_us()
        if value is not None and value < 255:
            values.append(value)
        time.sleep(0.03)
    return sorted(values)[len(values) // 2] if values else 255.0


# =============================================================================
# Grid
# =============================================================================
def tile_center(tile):
    return ((tile[0] + 0.5) * TILE, (tile[1] + 0.5) * TILE)


def tile_at(px, py):
    return (math.floor(px / TILE), math.floor(py / TILE))


def in_bounds(tile):
    return MIN_TILE_X <= tile[0] <= MAX_TILE_X and MIN_TILE_Y <= tile[1] <= MAX_TILE_Y


def neighbour(tile, heading):
    dx, dy = DIRS[heading % 360]
    return (tile[0] + dx, tile[1] + dy)


def heading_between(a, b):
    for h in DIRS:
        if neighbour(a, h) == tuple(b):
            return h
    raise ValueError(f"Tiles {a} and {b} are not next to each other")


def sensor_side(heading):
    """Direction the ultrasonic points when the robot faces `heading`."""
    return (heading + US_SIDE * 90) % 360


def other_side(heading):
    return (heading - US_SIDE * 90) % 360


def street_entrance(s1, s2):
    """
    The tiles on both sides of the s1-s2 edge: (approach, entry). The street
    is assumed to be on the side away from the origin.
    """
    (x1, y1), (x2, y2) = s1, s2
    if x1 == x2 and abs(y1 - y2) == 1:
        j = min(y1, y2)
        a, b = (x1 - 1, j), (x1, j)
    elif y1 == y2 and abs(x1 - x2) == 1:
        i = min(x1, x2)
        a, b = (i, y1 - 1), (i, y1)
    else:
        raise ValueError(f"s1={s1} and s2={s2} must be next to each other")
    if math.hypot(*tile_center(b)) < math.hypot(*tile_center(a)):
        a, b = b, a
    return a, b


def shortest_path(start, goal, avoid):
    """Tiles from start to goal around the tiles in `avoid` (fewest turns among the shortest)."""
    order = itertools.count()               # tie-breaker for the queue
    queue = [(0, next(order), start, None, [start])]
    done = set()
    while queue:
        cost, _, tile, h, path = heapq.heappop(queue)
        if tile == goal:
            return path
        if (tile, h) in done:
            continue
        done.add((tile, h))
        for nh in DIRS:
            nt = neighbour(tile, nh)
            if in_bounds(nt) and nt not in avoid:
                turn = 0.5 if h is not None and nh != h else 0
                heapq.heappush(queue, (cost + 1 + turn, next(order), nt, nh, path + [nt]))
    return None


# =============================================================================
# Correcting the odometry
# =============================================================================
def correct_from_bump(heading):
    """The bumper is on a box face, which lies on a grid line."""
    global x, y
    sign = 1 if heading in (0, 90) else -1
    centre = y if heading in (0, 180) else x
    bumper = centre + sign * BUMPER_OFFSET
    line = round(bumper / TILE) * TILE
    if abs(line - bumper) <= 8:
        if heading in (0, 180):
            y = line - sign * BUMPER_OFFSET
        else:
            x = line - sign * BUMPER_OFFSET


def correct_from_side_wall(tile, facing, distance, gain=1.0, max_away=8.0):
    """
    The ultrasonic reads `distance` to the box beside `tile`. Move the
    odometry across the street to agree. A slanted echo can only read too
    long, so a reading that puts us closer to the wall is trusted up to 8 cm;
    one that puts us farther away only up to `max_away`.
    """
    global x, y
    cx, cy = tile_center(tile)
    rad = math.radians(facing)
    rx, ry = math.cos(rad), -math.sin(rad)               # unit vector to the right
    across_odo = (x - cx) * rx + (y - cy) * ry
    tilt = math.radians(angle_diff(theta, facing))
    across_wall = US_SIDE * (TILE / 2 - (distance + US_SIDE_OFFSET) * math.cos(tilt))
    error = across_wall - across_odo
    closer = error * US_SIDE > 0
    if abs(error) <= (8.0 if closer else max_away):
        x += gain * error * rx
        y += gain * error * ry


def fix_heading_from_wall(facing):
    """
    Driving straight along a wall, the distance shrinks if we drift toward
    it: fit a line to distance vs. distance driven to get the true heading.
    """
    global theta
    s = [p[0] for p in wall_run]
    d = [p[1] for p in wall_run]
    n = len(s)
    if n < 6:
        return
    ms, md = sum(s) / n, sum(d) / n
    slope = sum((a - ms) * (b - md) for a, b in zip(s, d)) / sum((a - ms) ** 2 for a in s)
    spread = math.sqrt(sum((b - md - slope * (a - ms)) ** 2 for a, b in zip(s, d)) / n)
    odo_offset = sum(angle_diff(p[2], facing) for p in wall_run) / n
    wall_run.clear()
    if spread > 1.5 or abs(slope) > 0.3:               # noisy: don't trust it
        return
    error = -US_SIDE * math.degrees(math.asin(slope)) - odo_offset
    if abs(error) <= 10:
        theta = (theta + 0.7 * error) % 360


def follow_wall(tile, facing, reading):
    """Use one ultrasonic reading taken while driving beside `tile`."""
    if reading >= WALL_THRESHOLD or abs(angle_diff(theta, facing)) > 12:
        return                                   # no box there, or beam too slanted
    if reading >= 4:                             # closer readings are clipped
        wall_run.append((odo_dist, reading, theta))
    while wall_run and wall_run[0][0] < odo_dist - 30:
        wall_run.pop(0)                          # keep the last 30 cm
    # A slanted echo off the next box changes fast as we move, so a big move
    # away from the wall needs the last 4 cm of readings to agree.
    recent = [p for p in wall_run if p[0] >= odo_dist - 4]
    steady = (recent and odo_dist - recent[0][0] >= 3
              and max(p[1] for p in recent) - min(p[1] for p in recent) <= 2)
    correct_from_side_wall(tile, facing, reading, gain=0.25, max_away=8 if steady else 4)
    if wall_run and wall_run[-1][0] - wall_run[0][0] >= 15:
        fix_heading_from_wall(facing)


def tile_beside_sensor(facing):
    """
    The tile the ultrasonic is level with, or None when its beam would
    straddle two tiles (within 6 cm of a tile edge).
    """
    rad = math.radians(facing)
    sx = x + US_FORWARD_OFFSET * math.sin(rad)
    sy = y + US_FORWARD_OFFSET * math.cos(rad)
    along = sy if facing in (0, 180) else sx
    frac = (along / TILE) % 1
    return None if min(frac, 1 - frac) * TILE < 6 else tile_at(sx, sy)


def side_is_map_edge(tile, facing):
    return not in_bounds(neighbour(tile, sensor_side(facing)))


# =============================================================================
# Motion
# =============================================================================
def recentre():
    """
    The robot only just fits the street when it turns. If it is off the
    tile's centre line, back up a little and drive back: following the line
    pulls it onto the centre.
    """
    axis = nearest_axis(theta)
    if abs(angle_diff(axis, theta)) > 10:
        return
    cx, cy = tile_center(tile_at(x, y))
    ux, uy = DIRS[axis]
    across = (x - cx) * uy - (y - cy) * ux
    along = (x - cx) * ux + (y - cy) * uy
    if abs(across) <= 1.5 or abs(along) > TILE / 4:
        return
    print(f"  Re-centring ({across:+.1f} cm off the centre line)")
    drive_line((cx - 6 * ux, cy - 6 * uy), (axis + 180) % 360, reverse=True, dps=BACKUP_DPS)
    drive_line((cx, cy), axis, dps=PROBE_DPS)


def turn_to(target):
    """
    Turn in place. If the front catches a box while turning, back away a
    little and carry on (turning against a box ruins the heading).
    """
    target %= 360
    update_odometer()
    if abs(angle_diff(target, theta)) > 30:
        recentre()
    wall_run.clear()
    start, backoffs = time.time(), 0
    try:
        while abs(angle_diff(target, theta)) > 1.5 and time.time() - start < 6:
            if bumped() and backoffs < 4:
                backoffs += 1
                rad = math.radians(theta)
                drive_line((x - TURN_BACKOFF * math.sin(rad), y - TURN_BACKOFF * math.cos(rad)),
                           (theta + 180) % 360, reverse=True, dps=BACKUP_DPS)
                continue
            delta = angle_diff(target, theta)
            speed = clamp(abs(delta) * 5, 45, TURN_DPS)
            if delta > 0:
                set_wheels(speed, -speed)
            else:
                set_wheels(-speed, speed)
            time.sleep(0.01)
            update_odometer()
    finally:
        stop_motors()
        time.sleep(0.1)
        update_odometer()


def drive_line(target, heading, reverse=False, dps=DRIVE_DPS):
    """
    Drive along the line through `target` in direction `heading` and stop at
    `target`. Forward moves stop when the bumper is pressed. When moving
    along the grid, the side ultrasonic is read every loop (follow_wall).
    Returns OK, BUMP or FAIL (stuck for too long).
    """
    tx, ty = target
    if abs(angle_diff(nearest_axis(heading), heading)) < 3:
        heading = nearest_axis(heading)
    ux, uy = math.sin(math.radians(heading)), math.cos(math.radians(heading))
    facing = (heading + 180) % 360 if reverse else heading
    cm_per_s = dps / 360 * 2 * math.pi * RADIUS
    give_up = time.time() + 5 + 1.5 * math.hypot(tx - x, ty - y) / cm_per_s
    try:
        while True:
            update_odometer()
            dx, dy = tx - x, ty - y
            along = dx * ux + dy * uy                  # distance left
            if along <= 0.5:
                return OK
            if time.time() > give_up:
                print("Move failed (stuck?)")
                return FAIL
            if not reverse and bumped():
                print("Bump!")
                return BUMP

            if heading % 90 == 0:                      # along the grid
                reading = read_us()
                beside = tile_beside_sensor(facing)
                if beside and reading and not side_is_map_edge(beside, facing):
                    follow_wall(beside, facing, reading)

            speed = dps
            if along < 6:                               # slow down for the stop
                speed = max(min(STOP_DPS, dps), dps * along / 6)
            # Steer back onto the line (across > 0: we are right of it)
            across = -dx * uy + dy * ux
            desired = heading - math.degrees(math.atan2(across, 15))
            error = angle_diff(desired, theta + (180 if reverse else 0))
            steer = clamp(6 * error, -speed / 2, speed / 2)
            if reverse:
                set_wheels(-speed + steer, -speed - steer)
            else:
                set_wheels(speed + steer, speed - steer)
            time.sleep(0.01)
    finally:
        stop_motors()
        time.sleep(0.05)
        update_odometer()


def go_to(point):
    """Turn toward a point and drive to it. Returns OK, BUMP or FAIL."""
    dx, dy = point[0] - x, point[1] - y
    if math.hypot(dx, dy) < 1:
        return OK
    bearing = math.degrees(math.atan2(dx, dy)) % 360
    turn_to(bearing)
    return drive_line(point, bearing)


def back_up_to(point):
    """Reverse straight along the current axis until level with `point`."""
    return drive_line(point, (nearest_axis(theta) + 180) % 360, reverse=True, dps=BACKUP_DPS)


def follow_tiles(tiles):
    """Drive through the centres of a list of neighbouring tiles. Returns OK, BUMP or FAIL."""
    i = 0
    while i < len(tiles) - 1:
        # Drive each straight run in one go
        h = heading_between(tiles[i], tiles[i + 1])
        j = i + 1
        while j + 1 < len(tiles) and heading_between(tiles[j], tiles[j + 1]) == h:
            j += 1
        result = go_to(tile_center(tiles[j]))
        if result != OK:
            return result
        i = j
    return OK


# =============================================================================
# Sensing tiles
# =============================================================================
def probe_side_here(tile, facing):
    """Stopped at the centre of `tile`: is there a box on the sensor side?"""
    if side_is_map_edge(tile, facing):
        return True                       # the table edge bounds the street
    distance = read_us_median()
    is_wall = distance < WALL_THRESHOLD
    if is_wall:
        correct_from_side_wall(tile, facing, distance)
    print(f"  {NAMES[sensor_side(facing)]} of {tile}: {distance:5.1f} cm -> "
          f"{'WALL' if is_wall else 'open'}")
    return is_wall


def probe_front(tile, heading):
    """From the centre of `tile`, nudge forward 6 cm: True if the bumper hits a box."""
    cx, cy = tile_center(tile)
    dx, dy = DIRS[heading]
    if drive_line((cx + 6 * dx, cy + 6 * dy), heading, dps=PROBE_DPS) == BUMP:
        correct_from_bump(heading)
        back_up_to(tile_center(tile))
        print(f"  {NAMES[heading]}: bump -> WALL")
        return True
    print(f"  {NAMES[heading]}: open")
    return False


def drive_tile(tile, heading):
    """
    Drive from the centre of `tile` to the centre of the next tile along
    `heading` and look at its sensor side.

    Returns (result, side_wall), where side_wall says if the next tile has a
    box on the sensor side, and result is:
      OK     arrived at the centre of the next tile
      FRONT  arrived, and bumped the box in front of the next tile
      BUMP   the next tile is blocked (backed up to the centre of `tile`)
      FAIL
    """
    nxt = neighbour(tile, heading)
    result = drive_line(tile_center(nxt), heading)
    if result == BUMP:
        # The bump fixes where we are: were we still in `tile`, or already
        # in `nxt` (odometry behind) and touching the box in front of it?
        correct_from_bump(heading)
        if tile_at(x, y) == nxt:
            result = FRONT
            print(f"  {NAMES[heading]} of {nxt}: bump -> WALL")
            back_up_to(tile_center(nxt))
        else:
            print(f"  {NAMES[heading]} of {tile}: bump -> WALL")
            back_up_to(tile_center(tile))
            return BUMP, None
    if result == FAIL:
        return FAIL, None
    trail.append(nxt)
    return result, probe_side_here(nxt, heading)


# =============================================================================
# Phase 1
# =============================================================================
def go_to_approach(approach, entry):
    """
    Shortest path to the approach tile. A bump marks the tile in front as a
    box: back up and find another way. Returns True when there.
    """
    here = min(((0, 0), (-1, 0), (0, -1), (-1, -1)),
               key=lambda t: abs(t[0] - approach[0]) + abs(t[1] - approach[1]))
    go_to(tile_center(here))
    trail.append(here)
    boxes = {entry}                         # don't go through the street itself
    for _ in range(8):
        path = shortest_path(here, approach, boxes)
        if path is None:
            return False
        print(f"Path to the street: {path}")
        result = follow_tiles(path)
        if result == OK:
            trail.extend(path[1:])
            return True
        if result == FAIL:
            return False
        correct_from_bump(nearest_axis(theta))
        stopped = tile_at(x, y)
        if stopped not in path:
            return False
        box = neighbour(stopped, nearest_axis(theta))
        print(f"  Box at {box} - going around it")
        boxes.add(box)
        back_up_to(tile_center(stopped))
        trail.extend(path[1:path.index(stopped) + 1])
        here = stopped
    return False


def walk_street(approach, heading):
    """
    The robot is on `approach` facing the street. Map it.
    Returns (street, exit_tile):
      street      [{"tile": (i, j), "walls": {heading: is_wall}}] from the entrance
      exit_tile   the first open tile after the street, or None
    """
    street, visited = [], {approach}

    def blocked(t):
        return not in_bounds(t) or t in visited

    result, side_wall = drive_tile(approach, heading)
    if result in (BUMP, FAIL):
        print("Could not enter the street.")
        return street, None
    tile = neighbour(approach, heading)

    while True:
        visited.add(tile)
        side, other, ahead = sensor_side(heading), other_side(heading), neighbour(tile, heading)
        front_wall = result == FRONT
        print(f"Tile {tile} heading {NAMES[heading]}: {NAMES[side]} "
              f"{'WALL' if side_wall else 'open'}")

        if side_wall:
            # 1. Straight on (a bump means the front is a wall)
            if not front_wall and not blocked(ahead):
                result, next_wall = drive_tile(tile, heading)
                if result == FAIL:
                    print("Move failed - stopping mapping.")
                    return street, None
                if result != BUMP:
                    street.append({"tile": tile, "walls": {side: True, heading: False, other: True}})
                    tile, side_wall = ahead, next_wall
                    continue
            # 2. Front is a wall: turn to the other side, or it is a dead end
            turn_tile = neighbour(tile, other)
            if not blocked(turn_tile):
                turn_to(other)
                result, next_wall = drive_tile(tile, other)
                if result == FAIL:
                    print("Move failed - stopping mapping.")
                    return street, None
                if result != BUMP:
                    street.append({"tile": tile, "walls": {side: True, heading: True, other: False}})
                    tile, heading, side_wall = turn_tile, other, next_wall
                    continue
            street.append({"tile": tile, "walls": {side: True, heading: True, other: True}})
            print("Dead end: end of street.")
            return street, None

        # Sensor side open: the street turns that way (front and other side
        # are walls), or we have left the street.
        if front_wall:
            turns_here = True
        elif in_bounds(ahead):
            turns_here = probe_front(tile, heading)
        else:
            # The front is the table edge, which says nothing: turn around
            # and look at the other side with the ultrasonic instead.
            print(f"  {NAMES[heading]}: map edge")
            turn_to(heading + 180)
            turns_here = probe_side_here(tile, (heading + 180) % 360)
        if not turns_here:
            print(f"Left the street at {tile}.")
            return street, (tile if street else None)

        street.append({"tile": tile, "walls": {side: False, heading: True, other: True}})
        turn_tile = neighbour(tile, side)
        if blocked(turn_tile):
            print("Cannot continue (map edge).")
            return street, None
        turn_to(side)
        result, side_wall = drive_tile(tile, side)
        if result == BUMP:
            street[-1]["walls"][side] = True
            print("Dead end: end of street.")
            return street, None
        if result == FAIL:
            print("Move failed - stopping mapping.")
            return street, None
        tile, heading = turn_tile, side


def go_home():
    """Retrace every tile visited, in reverse, then go to (0, 0) and face +y."""
    print("\nGoing home")
    path = []
    for tile in reversed(trail):
        if not path or path[-1] != tile:
            path.append(tile)
    follow_tiles(path)
    go_to((0.0, 0.0))
    turn_to(0)
    print(f"Home: x={x:.1f} cm, y={y:.1f} cm, heading={theta:.1f} deg")


def run_phase1(s1, s2, d1, d2):
    approach, entry = street_entrance(s1, s2)
    heading_in = heading_between(approach, entry)
    print(f"Street entrance between {s1} and {s2}: approach {approach}, "
          f"entry {entry}, heading {NAMES[heading_in]}")

    street, exit_tile = [], None
    if go_to_approach(approach, entry):
        turn_to(heading_in)
        street, exit_tile = walk_street(approach, heading_in)
    else:
        print("Could not reach the approach tile.")
    go_home()

    street_map = {
        "s1": list(s1), "s2": list(s2), "d1": list(d1), "d2": list(d2),
        "approach_tile": list(approach),
        "entry_tile": list(entry),
        "entry_heading": heading_in,
        "street": [{"tile": list(s["tile"]),
                    "walls": {str(h): w for h, w in sorted(s["walls"].items())}}
                   for s in street],
        "exit_tile": list(exit_tile) if exit_tile else None,
        "street_length_tiles": len(street),
    }
    with open(MAP_FILE, "w") as f:
        json.dump(street_map, f, indent=2)
    print("Street tiles:", [s["tile"] for s in street])
    print("Exit tile:", exit_tile)
    return street_map


def parse_args(args):
    if not args:
        args = [input(f"{name} (x,y): ") for name in ("s1", "s2", "d1", "d2")]
    nums = [int(round(float(n))) for n in re.findall(r"-?\d+(?:\.\d+)?", " ".join(args))]
    if len(nums) != 8:
        print(__doc__)
        sys.exit("Expected 4 points: s1 s2 d1 d2 (each as x,y)")
    return tuple(tuple(nums[i:i + 2]) for i in range(0, 8, 2))


if __name__ == "__main__":
    try:
        run_phase1(*parse_args(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        stop_motors()
        reset_brick()
