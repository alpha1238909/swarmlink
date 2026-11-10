"""Small adapters for getting telemetry into SwarmLink from physical devices.

The decoder deliberately accepts only the normal, versioned SwarmLink packet
envelope.  Hardware can therefore be tested against the exact same packet
codec used by simulations and mission replay.
"""

from __future__ import annotations

from collections.abc import Iterator

from .core import NodeTelemetry
from .protocol import decode_packet


class SerialBridgeError(RuntimeError):
    """Raised when the optional pyserial dependency or a serial port is unavailable."""


class JSONLinesTelemetryDecoder:
    """Incrementally decode newline-delimited SwarmLink JSON packets.

    Devices may split USB serial output at arbitrary byte boundaries.  Feed
    every chunk received from a port to :meth:`feed`; incomplete data is kept
    until its newline arrives.  Invalid or oversized complete lines are
    counted and ignored so one bad radio packet cannot stop a live bridge.
    """

    def __init__(self, *, max_line_bytes: int = 1_024) -> None:
        if max_line_bytes < 64:
            raise ValueError("max_line_bytes must be at least 64")
        self.max_line_bytes = max_line_bytes
        self._buffer = bytearray()
        self.accepted_packets = 0
        self.rejected_packets = 0

    def feed(self, data: bytes | str) -> list[NodeTelemetry]:
        """Return every valid telemetry packet completed by *data*."""
        if isinstance(data, str):
            data = data.encode("utf-8")
        self._buffer.extend(data)
        decoded: list[NodeTelemetry] = []

        while True:
            try:
                newline = self._buffer.index(ord("\n"))
            except ValueError:
                # A device that never sends a delimiter must not make the
                # process consume unbounded memory. Drop its malformed frame.
                if len(self._buffer) > self.max_line_bytes:
                    self._buffer.clear()
                    self.rejected_packets += 1
                break

            line = bytes(self._buffer[:newline]).strip()
            del self._buffer[: newline + 1]
            if not line:
                continue
            if len(line) > self.max_line_bytes:
                self.rejected_packets += 1
                continue
            try:
                decoded.append(decode_packet(line))
                self.accepted_packets += 1
            except (UnicodeDecodeError, ValueError):
                self.rejected_packets += 1
        return decoded


def stream_serial_telemetry(
    port: str,
    *,
    baudrate: int = 115_200,
    read_size: int = 256,
) -> Iterator[NodeTelemetry]:
    """Yield telemetry from a USB serial bridge until the caller stops iterating.

    ``pyserial`` is intentionally optional so simulations do not gain a
    hardware dependency. Install it with ``pip install 'swarmlink[serial]'``.
    The device must print one normal ``encode_packet`` JSON message per line.
    """
    if baudrate <= 0 or read_size <= 0:
        raise ValueError("baudrate and read_size must be positive")
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as error:  # pragma: no cover - depends on environment
        raise SerialBridgeError("Serial support needs pyserial: pip install 'swarmlink[serial]'") from error

    decoder = JSONLinesTelemetryDecoder()
    try:
        with serial.Serial(port, baudrate=baudrate, timeout=1) as device:
            while True:
                for telemetry in decoder.feed(device.read(read_size)):
                    yield telemetry
    except serial.SerialException as error:  # pragma: no cover - physical port dependent
        raise SerialBridgeError(f"could not open serial port {port!r}: {error}") from error
