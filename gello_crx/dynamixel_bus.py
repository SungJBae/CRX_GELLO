"""Read-only DYNAMIXEL X-series bus for the GELLO leader arm.

Torque is explicitly disabled on every servo at open time and never enabled.
The XL330s are used purely as 12-bit absolute encoders, which keeps the
leader back-driveable.
"""

from __future__ import annotations

import numpy as np
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler

ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
TORQUE_DISABLE = 0
PROTOCOL_VERSION = 2.0
COUNTS_PER_REV = 4096
RAD_PER_COUNT = 2.0 * np.pi / COUNTS_PER_REV


class DynamixelBusError(RuntimeError):
    pass


class DynamixelBus:
    """Synchronous group read of present position for a fixed set of IDs."""

    def __init__(self, port: str, ids: list[int], baudrate: int = 57600):
        self._ids = list(ids)
        self._port = PortHandler(port)
        self._packet = PacketHandler(PROTOCOL_VERSION)

        if not self._port.openPort():
            raise DynamixelBusError(f"could not open {port}")
        if not self._port.setBaudRate(baudrate):
            raise DynamixelBusError(f"could not set baudrate {baudrate} on {port}")

        # Fail loudly if a servo is missing rather than silently returning zeros.
        for dxl_id in self._ids:
            _, comm, err = self._packet.ping(self._port, dxl_id)
            if comm != COMM_SUCCESS or err != 0:
                self._port.closePort()
                raise DynamixelBusError(
                    f"no response from DYNAMIXEL id {dxl_id} at {baudrate} baud"
                )
            self._packet.write1ByteTxRx(
                self._port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )

        self._reader = GroupSyncRead(
            self._port, self._packet, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )
        for dxl_id in self._ids:
            if not self._reader.addParam(dxl_id):
                raise DynamixelBusError(f"addParam failed for id {dxl_id}")

    def read_counts(self) -> np.ndarray:
        """Return raw counts for every id, or raise if the read failed."""
        if self._reader.txRxPacket() != COMM_SUCCESS:
            raise DynamixelBusError("group sync read failed")

        counts = np.zeros(len(self._ids), dtype=np.int64)
        for i, dxl_id in enumerate(self._ids):
            if not self._reader.isAvailable(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                raise DynamixelBusError(f"no data available for id {dxl_id}")
            counts[i] = self._reader.getData(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
        return counts

    def read_radians(self) -> np.ndarray:
        return self.read_counts().astype(np.float64) * RAD_PER_COUNT

    def close(self) -> None:
        self._port.closePort()
