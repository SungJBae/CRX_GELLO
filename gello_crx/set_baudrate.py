"""Change the baudrate of every DYNAMIXEL on the GELLO leader bus.

Run this ONCE before switching gello_crx.yaml to a higher baudrate:

    # Flash all 7 servos from 57600 to 2 Mbps
    python3 -m gello_crx.set_baudrate \
        --port /dev/ttyUSB0 \
        --current-baud 57600 \
        --target-baud 2000000

After the script succeeds, update the baudrate field in
config/gello_crx.yaml and re-run colcon build.

Baudrate table (EEPROM address 8):
    0 ->    9 600
    1 ->   57 600   (factory default)
    2 ->  115 200
    3 -> 1 000 000
    4 -> 2 000 000  (recommended)
    5 -> 3 000 000
    6 -> 4 000 000
"""

from __future__ import annotations

import argparse
import sys

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_BAUD_RATE = 8
ADDR_TORQUE_ENABLE = 64
PROTOCOL_VERSION = 2.0

BAUD_TABLE = {
    9600: 0,
    57600: 1,
    115200: 2,
    1000000: 3,
    2000000: 4,
    3000000: 5,
    4000000: 6,
}


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Change DYNAMIXEL baudrate for every servo on the bus."
    )
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument(
        "--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6, 7],
        help="Servo IDs to update (default: 1-7).",
    )
    ap.add_argument(
        "--current-baud", type=int, default=57600,
        help="Current baudrate the servos are set to.",
    )
    ap.add_argument(
        "--target-baud", type=int, default=2000000,
        help="New baudrate to write (default: 2 000 000).",
    )
    args = ap.parse_args()

    if args.current_baud not in BAUD_TABLE:
        sys.exit(f"current-baud {args.current_baud} not in {list(BAUD_TABLE)}")
    if args.target_baud not in BAUD_TABLE:
        sys.exit(f"target-baud {args.target_baud} not in {list(BAUD_TABLE)}")
    if args.current_baud == args.target_baud:
        sys.exit("current and target baudrate are the same, nothing to do")

    target_value = BAUD_TABLE[args.target_baud]

    port = PortHandler(args.port)
    pkt = PacketHandler(PROTOCOL_VERSION)

    if not port.openPort():
        sys.exit(f"could not open {args.port}")
    if not port.setBaudRate(args.current_baud):
        sys.exit(f"could not set baudrate {args.current_baud}")

    print(f"connected at {args.current_baud} baud on {args.port}")

    for dxl_id in args.ids:
        _, comm, err = pkt.ping(port, dxl_id)
        if comm != COMM_SUCCESS or err != 0:
            print(f"  [SKIP] id {dxl_id}: no response at {args.current_baud} baud")
            continue

        # Torque must be disabled to write EEPROM
        pkt.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)

        # Write the new baudrate value
        _, comm, err = pkt.write1ByteTxRx(port, dxl_id, ADDR_BAUD_RATE, target_value)
        if comm != COMM_SUCCESS or err != 0:
            print(f"  [FAIL] id {dxl_id}: write error (comm={comm}, err={err})")
        else:
            print(f"  [OK]   id {dxl_id}: baudrate set to {args.target_baud}")

    port.closePort()

    # Verify by reopening at the new baudrate
    print(f"\nverifying at {args.target_baud} baud ...")
    if not port.openPort():
        sys.exit(f"could not reopen {args.port}")
    if not port.setBaudRate(args.target_baud):
        sys.exit(f"could not set baudrate {args.target_baud}")

    ok = 0
    for dxl_id in args.ids:
        _, comm, err = pkt.ping(port, dxl_id)
        if comm != COMM_SUCCESS or err != 0:
            print(f"  [FAIL] id {dxl_id}: no response at {args.target_baud}")
        else:
            print(f"  [OK]   id {dxl_id}: responds at {args.target_baud}")
            ok += 1

    port.closePort()
    print(f"\n{ok}/{len(args.ids)} servos verified at {args.target_baud} baud")
    if ok < len(args.ids):
        print("WARNING: some servos did not respond. Power-cycle them and retry.")
        sys.exit(1)
    else:
        print("All servos updated. Update baudrate in config/gello_crx.yaml to match.")


if __name__ == "__main__":
    main()
