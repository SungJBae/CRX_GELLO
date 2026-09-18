"""Interactive calibration for the GELLO leader.

Produces the joint_offsets, joint_signs and trigger endpoints that go into
config/gello_crx.yaml. Run it with the robot NOT connected; this only talks to
the leader bus.

    ros2 run gello_crx gello_calibrate --port /dev/ttyUSB0

Three stages:
  signs    move one joint at a time, confirm which way is positive
  zero     hold the leader in the CRX zero pose, capture offsets
  trigger  squeeze and release, capture the trigger endpoints
"""

from __future__ import annotations

import argparse

import numpy as np

from gello_crx.dynamixel_bus import RAD_PER_COUNT, DynamixelBus

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]

# Which way each CRX joint rotates positively, described in plain language so
# you can check the leader against the robot without a teach pendant.
POSITIVE_SENSE = {
    "J1": "base rotates counter-clockwise seen from above",
    "J2": "shoulder pitches the upper arm forward",
    "J3": "elbow pitches the forearm up",
    "J4": "forearm rolls counter-clockwise seen from the base",
    "J5": "wrist pitches the tool up",
    "J6": "tool rolls counter-clockwise seen from behind the flange",
}


def _median_counts(bus: DynamixelBus, samples: int = 25) -> np.ndarray:
    readings = []
    while len(readings) < samples:
        try:
            readings.append(bus.read_counts())
        except Exception:  # noqa: BLE001 - a dropped packet is not fatal here
            continue
    return np.median(np.stack(readings), axis=0)


def stage_signs(bus: DynamixelBus) -> np.ndarray:
    print("\n=== SIGNS ===")
    print("For each joint: move it in the direction described, then press Enter.")
    print("Move at least 20 degrees so the reading is unambiguous.\n")
    signs = np.ones(6)
    for i, name in enumerate(JOINTS):
        before = _median_counts(bus)[i]
        input(f"  {name}: {POSITIVE_SENSE[name]} ... then Enter ")
        after = _median_counts(bus)[i]
        delta = ((after - before + 2048) % 4096) - 2048  # shortest signed arc
        if abs(delta) < 100:
            print(f"    only {abs(delta) * RAD_PER_COUNT * 57.3:.1f} deg of travel "
                  f"- too small to trust, assuming +1. Re-run if this joint misbehaves.")
            signs[i] = 1.0
        else:
            signs[i] = 1.0 if delta > 0 else -1.0
            print(f"    moved {delta * RAD_PER_COUNT * 57.3:+.1f} deg "
                  f"-> sign {signs[i]:+.0f}")
    return signs


def stage_zero(bus: DynamixelBus, signs: np.ndarray) -> np.ndarray:
    print("\n=== ZERO ===")
    print("Put the leader in the CRX zero pose: upper arm straight up, forearm")
    print("horizontal and pointing forward, wrist straight.")
    print("Use your printed calibration cradle if you built one.\n")
    input("  Hold the pose, then press Enter ")
    counts = _median_counts(bus, samples=50)
    # mapped = sign * (raw - offset); we want mapped == 0 in this pose
    offsets = counts[:6] * RAD_PER_COUNT
    for i, name in enumerate(JOINTS):
        print(f"    {name}: raw {counts[i]:6.0f} counts -> offset {offsets[i]:+.4f} rad")
    _warn_on_seam(counts[:6])
    return offsets


def _warn_on_seam(counts: np.ndarray) -> None:
    """Flag joints whose zero sits close to the 0/4095 wrap."""
    near = [
        JOINTS[i]
        for i, c in enumerate(counts)
        if c < 683 or c > 3413  # within 60 deg of the seam
    ]
    if near:
        print(
            "\n  WARNING: "
            + ", ".join(near)
            + " sit within 60 deg of the 0/4095 seam at the zero pose."
        )
        print("  Re-clock those horns before you trust the mapping.")


def stage_trigger(bus: DynamixelBus) -> tuple[int, int]:
    print("\n=== TRIGGER ===")
    input("  Release the trigger fully (gripper open), then Enter ")
    open_counts = int(_median_counts(bus)[6])
    input("  Squeeze the trigger fully (gripper closed), then Enter ")
    closed_counts = int(_median_counts(bus)[6])
    print(f"    open {open_counts}, closed {closed_counts}, "
          f"span {abs(closed_counts - open_counts)} counts")
    if abs(closed_counts - open_counts) < 200:
        print("  WARNING: span under 200 counts gives coarse gripper control.")
    return open_counts, closed_counts


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baudrate", type=int, default=57600)
    ap.add_argument("--joint-ids", type=int, nargs=6, default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--trigger-id", type=int, default=7)
    args = ap.parse_args()

    bus = DynamixelBus(args.port, list(args.joint_ids) + [args.trigger_id], args.baudrate)
    try:
        signs = stage_signs(bus)
        offsets = stage_zero(bus, signs)
        trig_open, trig_closed = stage_trigger(bus)
    finally:
        bus.close()

    print("\n=== PASTE INTO config/gello_crx.yaml ===\n")
    print("/**/gello_leader:")
    print("  ros__parameters:")
    print(f"    joint_offsets: [{', '.join(f'{v:.4f}' for v in offsets)}]")
    print(f"    joint_signs: [{', '.join(f'{v:.1f}' for v in signs)}]")
    print(f"    trigger_open_counts: {trig_open}")
    print(f"    trigger_closed_counts: {trig_closed}")
    print()


if __name__ == "__main__":
    main()
