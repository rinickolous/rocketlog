import time
from collections import deque

import serial

from rocketlog.telemetry import protocol
from rocketlog.telemetry.events import AckEvent, ReceiverLogEvent, TelemetryEvent, TelemetryStreamEvent
from rocketlog.telemetry.types import Telemetry


class TelemetryReader:
    """Telemetry + receiver-log reader using framed binary protocol."""

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)

        # Drain any partial data at startup
        self._ser.reset_input_buffer()

        self._rx_buf = bytearray()
        self._pending: deque[TelemetryStreamEvent] = deque()

        self._seq = 1
        self._send_time_sync()

    # ---------------------------------------- #
    #  Connection                              #
    # ---------------------------------------- #

    def close(self) -> None:
        self._ser.close()

    # ---------------------------------------- #

    def _send_time_sync(self) -> None:
        seq = self._seq
        self._seq += 1

        t = protocol.now_unix()
        pkt = protocol.encode_time_sync(t, seq)
        frame = protocol.cobs_encode(pkt) + b"\x00"
        self._ser.write(frame)
        self._ser.flush()

        deadline = time.time() + 5.0
        while time.time() < deadline:
            evt = self.read_event(timeout_s=0.2)
            if evt is None:
                continue
            if isinstance(evt, ReceiverLogEvent):
                # Surface receiver logs during handshake to aid debugging.
                print("RX_LOG:", protocol.format_log(evt.log.level, evt.log.msg))

            if isinstance(evt, AckEvent) and evt.ack_type == protocol.MSG_TIME_SYNC and evt.seq == seq:
                if evt.status != protocol.ACK_STATUS_OK:
                    raise RuntimeError(f"Time sync rejected (status={evt.status})")
                return

        raise TimeoutError("Time sync ACK not received")

    # ---------------------------------------- #

    def read_event(self, timeout_s: float = 0.0) -> TelemetryStreamEvent | None:
        if self._pending:
            return self._pending.popleft()

        end_time = time.time() + max(0.0, timeout_s)
        while True:
            if self._pending:
                return self._pending.popleft()

            if timeout_s == 0.0:
                read_deadline = time.time()
            else:
                read_deadline = end_time

            if time.time() > read_deadline:
                return None

            chunk = self._ser.read(512)
            if not chunk:
                continue

            self._rx_buf.extend(chunk)
            self._drain_frames(host_time=time.time())

    # ---------------------------------------- #

    def _drain_frames(self, host_time: float) -> None:
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
                # Ignore malformed frames; COBS delimiter makes resync cheap.
                continue

            if msg_type == protocol.MSG_TELEMETRY:
                try:
                    t_unix, alt_m, vel_mps, batt_v, temp_c = protocol.decode_telemetry_payload(payload)
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
                        )
                    )
                )
                continue

            if msg_type == protocol.MSG_LOG:
                try:
                    level, t_unix_receiver, msg = protocol.decode_log_payload(payload)
                except Exception:
                    continue

                self._pending.append(
                    ReceiverLogEvent(
                        protocol.ReceiverLog(
                            t_unix_receiver=t_unix_receiver,
                            t_unix_host=host_time,
                            level=level,
                            msg=msg,
                        )
                    )
                )
                continue

            if msg_type == protocol.MSG_ACK:
                try:
                    ack_type, seq, status, applied = protocol.decode_ack_payload(payload)
                except Exception:
                    continue

                self._pending.append(AckEvent(ack_type=ack_type, seq=seq, status=status, applied_time_unix=applied))
                continue

            # Unknown message types are ignored.

    # ---------------------------------------- #

    def ping(self) -> None:
        # legacy no-op; framed link may add PING later
        return

    # ---------------------------------------- #

    def sample(self) -> Telemetry:
        """Return the next telemetry sample; receiver logs are skipped."""
        while True:
            evt = self.read_event(timeout_s=1.0)
            if evt is None:
                continue
            if isinstance(evt, TelemetryEvent):
                return evt.telemetry

    def receiver_log(self) -> protocol.ReceiverLog | None:
        """Return the next receiver log message if available."""
        evt = self.read_event(timeout_s=0.0)
        if isinstance(evt, ReceiverLogEvent):
            return evt.log
        return None
