"""
uart_tx.py — 8-Byte Little-Endian vision packet serializer + UART transmitter
=============================================================================

Immutable data contract (GEMINI.md §2), shared with stm32_fw/include/protocol.h:

    Byte 0   : 0xFF            (Header 1)
    Byte 1   : 0xFE            (Header 2)
    Byte 2   : uint8  status   (0x00 lost, 0x01 detected, 0x02 coasting)
    Byte 3-4 : int16  err_x    (LE, -960 .. +960)
    Byte 5-6 : int16  err_y    (LE, -540 .. +540)
    Byte 7   : uint8  checksum = (status + ex_L + ex_H + ey_L + ey_H) & 0xFF

Link: Raspberry Pi 5 /dev/ttyAMA0 (GPIO14/15) @ 115200 bps 8N1 -> STM32 USART1.
NOTE: disable the Linux serial console on ttyAMA0 (raspi-config) so boot logs
      do not reach the STM32.
"""

from __future__ import annotations

import struct
from typing import Optional, Tuple

HEADER1 = 0xFF
HEADER2 = 0xFE
PACKET_SIZE = 8

DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUDRATE = 115200

LIMIT_X_PX = 960
LIMIT_Y_PX = 540
VALID_STATUS = (0x00, 0x01, 0x02)

# '<' = little-endian, no padding: B B B h h B = 1+1+1+2+2+1 = 8 bytes
_PACKET_STRUCT = struct.Struct("<BBBhhB")
assert _PACKET_STRUCT.size == PACKET_SIZE


def compute_checksum(status: int, err_x: int, err_y: int) -> int:
    """Checksum exactly as defined in the data contract."""
    ux = err_x & 0xFFFF
    uy = err_y & 0xFFFF
    return (status
            + (ux & 0xFF) + ((ux >> 8) & 0xFF)
            + (uy & 0xFF) + ((uy >> 8) & 0xFF)) & 0xFF


def build_packet(status: int, err_x: int, err_y: int) -> bytes:
    """Serialize one 8-byte packet (values are clamped to the contract range)."""
    status = int(status) & 0xFF
    err_x = max(-LIMIT_X_PX, min(LIMIT_X_PX, int(err_x)))
    err_y = max(-LIMIT_Y_PX, min(LIMIT_Y_PX, int(err_y)))
    chk = compute_checksum(status, err_x, err_y)
    return _PACKET_STRUCT.pack(HEADER1, HEADER2, status, err_x, err_y, chk)


def parse_packet(data: bytes) -> Optional[Tuple[int, int, int]]:
    """Decode an aligned 8-byte packet. Returns (status, err_x, err_y) or None."""
    if len(data) != PACKET_SIZE:
        return None
    h1, h2, status, ex, ey, chk = _PACKET_STRUCT.unpack(data)
    if h1 != HEADER1 or h2 != HEADER2:
        return None
    if chk != compute_checksum(status, ex, ey):
        return None
    return status, ex, ey


class UartTransmitter:
    """Non-blocking-ish UART sender for vision packets (pyserial)."""

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE,
                 write_timeout: float = 0.02, dry_run: bool = False) -> None:
        self.port = port
        self.baudrate = baudrate
        self.dry_run = dry_run
        self.sent_count = 0
        self.drop_count = 0
        self.last_packet: bytes = b""
        self._serial = None

        if not dry_run:
            import serial  # pyserial — imported lazily so tests run without it
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=write_timeout,
            )

    def send(self, status: int, err_x: int, err_y: int) -> bytes:
        pkt = build_packet(status, err_x, err_y)
        self.last_packet = pkt
        if self._serial is not None:
            try:
                self._serial.write(pkt)
            except Exception:  # SerialTimeoutException / OSError -> drop frame
                self.drop_count += 1
                return pkt
        self.sent_count += 1
        return pkt

    def close(self) -> None:
        if self._serial is not None:
            try:
                # Leave the STM32 in a safe "target lost" state.
                self._serial.write(build_packet(0x00, 0, 0))
                self._serial.flush()
            except Exception:
                pass
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "UartTransmitter":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
