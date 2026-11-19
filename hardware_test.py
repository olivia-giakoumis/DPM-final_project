#!/usr/bin/env python3
"""
ECSE 211 hardware test (utils.brick version).

Tests:
  1. Motor on port C (clockwise or counterclockwise)
  2. EV3 colour sensor
  3. NXT colour sensor
  4. Touch sensor
  5. Ultrasonic sensor

Stops on any error or Ctrl+C, and always stops the motor and resets the brick.
"""

import time

from utils.brick import (
    Motor,
    TouchSensor,
    EV3UltrasonicSensor,
    EV3ColorSensor,
    NXTColorSensor,
    reset_brick,
    wait_ready_sensors,
)

# ---------------- Configuration (change to match your wiring) ----------------
MOTOR_PORT = "C"

TOUCH_PORT = 1
ULTRASONIC_PORT = 2
EV3_COLOR_PORT = 3
NXT_COLOR_PORT = 4

# If "cw" spins the wrong way, flip this to -1.
CW_SIGN = 1

# Black-line thresholds (same values as lab 2; retune for your sensors)
EV3_THRESHOLD = 25   # EV3 get_red() below this = black
# NXT brightness (average of R, G, B). The scale is different from get_value(),
# so run the NXT test over a white tile and a black line and use the suggested
# threshold it prints at the end.
NXT_THRESHOLD = 300

READ_SECONDS = 10       # how long each sensor test prints readings
READ_INTERVAL = 0.25    # seconds between readings
# ------------------------------------------------------------------------------

motor = Motor(MOTOR_PORT)


def stop():
    motor.set_dps(0)


def read_loop(name, read_fn, fmt):
    """Print sensor readings for READ_SECONDS."""
    print(f"Reading {name} for {READ_SECONDS} s (Ctrl+C to quit)...")
    end = time.time() + READ_SECONDS
    while time.time() < end:
        value = read_fn()
        if value is None:
            print("  no reading (sensor not ready or bad read)")
        else:
            print(f"  {fmt(value)}")
        time.sleep(READ_INTERVAL)


# ------------------------------- Tests ----------------------------------------
def test_motor():
    direction = input("Direction? [cw/ccw]: ").strip().lower()
    if direction not in ("cw", "ccw"):
        raise ValueError(f"Invalid direction '{direction}'. Use cw or ccw.")

    speed = float(input("Speed in degrees/second (e.g. 360): "))
    duration = float(input("Duration in seconds (e.g. 2): "))
    if speed <= 0 or duration <= 0:
        raise ValueError("Speed and duration must be positive.")

    sign = CW_SIGN if direction == "cw" else -CW_SIGN

    motor.reset_encoder()
    print(f"Spinning motor {MOTOR_PORT} {direction} at {speed} dps for {duration} s...")

    try:
        motor.set_dps(sign * speed)
        end = time.time() + duration
        while time.time() < end:
            print(f"  encoder: {motor.get_encoder()} deg", end="\r")
            time.sleep(0.1)
    finally:
        stop()

    print(f"\nDone. Total rotation: {motor.get_encoder()} deg "
          f"(expected about {sign * speed * duration:.0f})")


def first_value(v):
    """Some reads come back as a list; take the first element (as in lab 2)."""
    if isinstance(v, (list, tuple)):
        return v[0] if len(v) > 0 else None
    return v


def test_ev3_color():
    sensor = EV3ColorSensor(EV3_COLOR_PORT)
    wait_ready_sensors()
    read_loop(
        "EV3 colour sensor (get_red)",
        lambda: first_value(sensor.get_red()),
        lambda v: f"red = {v:4}  -> {'BLACK' if v < EV3_THRESHOLD else 'white'}"
                  f"  (threshold {EV3_THRESHOLD})",
    )


def nxt_brightness(sensor):
    """Return (brightness, detail string) from the NXT sensor.

    get_value() in full-colour mode is a colour ID (1-7), not intensity,
    so use the RGB channels instead.
    """
    if hasattr(sensor, "get_rgb"):
        rgb = sensor.get_rgb()
        if rgb is None or None in rgb[:3]:
            return None, ""
        r, g, b = rgb[:3]
        return (r + g + b) / 3, f"R={r:4} G={g:4} B={b:4}"
    if hasattr(sensor, "get_red"):
        r = first_value(sensor.get_red())
        return r, f"red={r}"
    raise AttributeError(
        "NXTColorSensor has no get_rgb or get_red. Available methods: "
        + ", ".join(m for m in dir(sensor) if m.startswith("get"))
    )


def test_nxt_color():
    sensor = NXTColorSensor(NXT_COLOR_PORT)
    wait_ready_sensors()

    lo, hi = None, None
    print(f"Reading NXT colour sensor for {READ_SECONDS} s. "
          "Move it between a white tile and a black line.")
    end = time.time() + READ_SECONDS
    while time.time() < end:
        value, detail = nxt_brightness(sensor)
        if value is None:
            print("  no reading (sensor not ready or bad read)")
        else:
            lo = value if lo is None else min(lo, value)
            hi = value if hi is None else max(hi, value)
            state = "BLACK" if value < NXT_THRESHOLD else "white"
            print(f"  {detail}  brightness={value:6.1f}  -> {state}  "
                  f"(threshold {NXT_THRESHOLD})")
        time.sleep(READ_INTERVAL)

    if lo is not None:
        print(f"\nMin brightness: {lo:.1f} | Max brightness: {hi:.1f}")
        print(f"Suggested NXT_THRESHOLD (midpoint): {(lo + hi) / 2:.0f}")


def test_touch():
    sensor = TouchSensor(TOUCH_PORT)
    wait_ready_sensors()
    read_loop(
        "touch sensor",
        sensor.is_pressed,
        lambda v: "PRESSED" if v else "released",
    )


def test_ultrasonic():
    sensor = EV3UltrasonicSensor(ULTRASONIC_PORT)
    wait_ready_sensors()
    read_loop(
        "ultrasonic sensor",
        sensor.get_cm,
        lambda v: f"distance = {v:.1f} cm",
    )


TESTS = {
    "1": (f"Motor (port {MOTOR_PORT})", test_motor),
    "2": ("EV3 colour sensor", test_ev3_color),
    "3": ("NXT colour sensor", test_nxt_color),
    "4": ("Touch sensor", test_touch),
    "5": ("Ultrasonic sensor", test_ultrasonic),
}


def main():
    while True:
        print("\n=== Hardware test menu ===")
        for key, (label, _) in TESTS.items():
            print(f"  {key}. {label}")
        print("  q. Quit")

        choice = input("Choose a test: ").strip().lower()
        if choice == "q":
            break
        if choice not in TESTS:
            print("Invalid choice, try again.")
            continue

        label, func = TESTS[choice]
        print(f"\n--- {label} ---")
        func()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Stopping.")
    except BaseException as e:
        print(f"\nError: {e!r}")
    finally:
        stop()
        reset_brick()
        print("Motor stopped and brick reset.")
