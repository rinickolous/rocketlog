from __future__ import annotations

import time
from collections import deque

import serial

from rocketlog.telemetry import protocol
from rocketlog.telemetry.events import (
    AckEvent,
    ReceiverLogEvent,
    TelemetryEvent,
    TelemetryStreamEvent,
)
from rocketlog.telemetry.protocol import ReceiverLog
from rocketlog.telemetry.types import Telemetry


# ---------------------------------------- #
# TelemetryReader                           #
# ---------------------------------------- #


class TelemetryReader:
    """
    Synchronous serial reader for the RocketLog binary protocol.

    Opens the serial port on construction, performs the TIME_SYNC handshake,
    then provides event-driven access to incoming packets.

    Raises
    ------
    TimeoutError  if the time-sync ACK is not received within 5 seconds.
    RuntimeError  if the firmware rejects the time-sync.
    serial.SerialException  if the port cannot be opened.
    """

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._ser.reset_input_buffer()
        self._rx_buf = bytearray()
        self._pending: deque[TelemetryStreamEvent] = deque()
        self._seq = 1
        self._send_time_sync()

    # ---------------------------------------- #

    def close(self) -> None:
        """Close the serial port."""
        try:
            self._ser.close()
        except Exception:
            pass

    # ---------------------------------------- #

    def _send_time_sync(self) -> None:
        """Send a TIME_SYNC packet and wait for the matching ACK."""
        seq = self._seq
        self._seq += 1
        pkt = protocol.encode_time_sync(protocol.now_unix(), seq)
        frame = protocol.cobs_encode(pkt) + b"\x00"
        self._ser.write(frame)
        self._ser.flush()

        deadline = time.time() + 5.0
        while time.time() < deadline:
            evt = self.read_event(timeout_s=0.2)
            if evt is None:
                continue
            if isinstance(evt, ReceiverLogEvent):
                # Surface firmware logs during the handshake for diagnostics.
                print("RX LOG:", protocol.format_log(evt.log.level, evt.log.msg))
            if (
                isinstance(evt, AckEvent)
                and evt.ack_type == protocol.MSG_TIME_SYNC
                and evt.seq == seq
            ):
                if evt.status != protocol.ACK_STATUS_OK:
                    raise RuntimeError(f"Time sync rejected (status={evt.status})")
                return

        raise TimeoutError("Time sync ACK not received within 5 s")

    # ---------------------------------------- #

    def read_event(self, timeout_s: float = 0.0) -> TelemetryStreamEvent | None:
        """
        Return the next pending event, or None if none arrive within timeout_s.

        Reads available bytes from serial, decodes COBS frames, and fills the
        internal pending queue. Returns immediately (timeout_s=0) for non-blocking
        use or blocks for up to timeout_s seconds.
        """
        if self._pending:
            return self._pending.popleft()

        end_time = time.time() + max(0.0, timeout_s)
        while True:
            if self._pending:
                return self._pending.popleft()

            if time.time() > end_time:
                return None

            chunk = self._ser.read(512)
            if not chunk:
                continue

            self._rx_buf.extend(chunk)
            self._drain_frames(host_time=time.time())

    # ---------------------------------------- #

    def _drain_frames(self, host_time: float) -> None:
        """Split rx buffer on 0x00 delimiters and decode each complete COBS frame."""
        while True:
            try:
                idx = self._rx_buf.index(0)
            except ValueError:
                return

            frame = bytes(self._rx_buf[:idx])
            del self._rx_buf[: idx + 1]

            if not frame:
                continue

            try:
                pkt = protocol.cobs_decode(frame)
                msg_type, payload = protocol.decode_packet(pkt)
            except Exception:
                # Silently skip malformed frames; COBS delimiter makes resync trivial.
                continue

            match msg_type:
                case protocol.MSG_TELEMETRY:
                    try:
                        t_unix, alt_m, vel_mps, batt_v, temp_c = (
                            protocol.decode_telemetry_payload(payload)
                        )
                    except Exception:
                        continue
                    self._pending.append(
                        TelemetryEvent(
                            Telemetry(
                                t_unix=t_unix,
                                alt_m=alt_m,
                                vel_mps=vel_mps,
                                batt_v=batt_v,
                                temp_c=temp_c,
                                pressure_pa=0.0,
                                lat=0.0,
                                lon=0.0,
                                gps_alt_m=0.0,
                                gps_sats=0,
                                gps_fix=False,
                            )
                        )
                    )

                case protocol.MSG_LOG:
                    try:
                        level, t_unix_receiver, msg = protocol.decode_log_payload(payload)
                    except Exception:
                        continue
                    self._pending.append(
                        ReceiverLogEvent(
                            ReceiverLog(
                                t_unix_receiver=t_unix_receiver,
                                t_unix_host=host_time,
                                level=level,
                                msg=msg,
                            )
                        )
                    )

                case protocol.MSG_ACK:
                    try:
                        ack_type, seq, status, applied = protocol.decode_ack_payload(payload)
                    except Exception:
                        continue
                    self._pending.append(
                        AckEvent(
                            ack_type=ack_type,
                            seq=seq,
                            status=status,
                            applied_time_unix=applied,
                        )
                    )

                # Unknown message types are silently ignored.

    # ---------------------------------------- #

    def sample(self) -> Telemetry:
        """Block until the next telemetry packet arrives (skips log/ack events)."""
        while True:
            evt = self.read_event(timeout_s=1.0)
            if isinstance(evt, TelemetryEvent):
                return evt.telemetry

    # ---------------------------------------- #

    def receiver_log(self) -> ReceiverLog | None:
        """Return the next buffered log message without blocking, or None."""
        evt = self.read_event(timeout_s=0.0)
        if isinstance(evt, ReceiverLogEvent):
            return evt.log
        return None
