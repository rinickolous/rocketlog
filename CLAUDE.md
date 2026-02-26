# RocketLog — Claude Context

## Project Summary

RocketLog is a student rocketry ground station. Two ESP32-S3 transmitters fly onboard — one in a CanSat, one in the rocket body — sending telemetry over LoRa to a USB-connected receiver. The receiver relays data to a PySide6 desktop app that displays live video, telemetry, and recordings.

The ground station runs on a cyberdeck built around a Raspberry Pi 3 (Debian/PiOS). Target altitude ~2000m.

---

## Repository Layout

```
rocketlog/
├── src/rocketlog/          # Active Python app (WIP rewrite — owner's code)
├── src/rocketlog.old/      # Reference implementation (GPT-generated, being replaced)
├── firmware/
│   ├── receiver/           # ESP-IDF project: USB serial relay (simulated telemetry for now)
│   ├── transmitter/        # ESP-IDF project: real sensor readout (not yet protocol-wired)
│   ├── components/
│   │   ├── common/         # Board pin defs, monotonic time API
│   │   ├── sensors/        # GPS (NMEA/UART) + MPL3115A2 barometer (I2C) drivers
│   │   └── telemetry_protocol/  # Binary packet protocol (COBS framing, CRC32)
│   └── flash.toml          # Serial port config (receiver=/dev/ttyACM1, transmitter=/dev/ttyACM0)
├── justfile                # All build/flash/run commands
├── pyproject.toml
└── compile_commands.json   # Symlink → active firmware build (for clangd)
```

---

## Code Ownership

- **C firmware**: ~90% owner-written. Treat as authoritative — don't refactor without prompting.
- **Python `src/rocketlog/`**: owner-written rewrite. Actively being developed.
- **Python `src/rocketlog.old/`**: read-only reference (GPT-generated). Use it to understand existing behaviour, not as a style guide.

---

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | ESP32-S3 (Xtensa LX7) |
| Barometer | MPL3115A2 — I2C port 0, SCL=GPIO11, SDA=GPIO12, 40 kHz |
| GPS | GY-GPS6MV2 (u-blox NEO-6M) — UART1, 9600 baud, TX=GPIO17, RX=GPIO16 |
| LED | SK6812 RGB — RMT, GPIO48 |
| Radio | LoRa (not yet wired in firmware) |
| Ground station | Raspberry Pi 3, Debian/PiOS |

---

## Binary Protocol

All wire data is little-endian, packed, no floats.

**Packet layout** (COBS-framed, `0x00` = frame delimiter):
```
magic[2]      : 0x52 0x4C  ("RL")
version       : u8  = 1
msg_type      : u8  (TELEMETRY=1, LOG=2, TIME_SYNC=3, ACK=4)
payload_len   : u8
payload       : [payload_len bytes]
crc32         : u32 (little-endian, covers all prior bytes)
```

**Telemetry payload** (29 bytes total packet, `<qiiHh` struct):
```
unix_time_us   : i64  (UTC microseconds)
altitude_cm    : i32
velocity_cms   : i32
battery_mv     : u16
temperature_cC : i16  (centi-°C)
```

**Time sync flow**: Python sends `TIME_SYNC` → firmware applies offset and replies `ACK` → firmware emits timestamped telemetry.

CRC32 uses standard polynomial — `esp_crc32_le()` in C, `zlib.crc32()` in Python.

---

## Firmware State

| Project | Status |
|---------|--------|
| `receiver` | Simulates fake ascent/descent curve, sends binary-framed telemetry over USB JTAG serial at 10 Hz. Handles TIME_SYNC. LoRa RX is stubbed. |
| `transmitter` | Reads real MPL3115A2 + GPS sensors. Logs via `ESP_LOG` only — not yet sending binary protocol frames. |

Known issue: `sensor_gps.c:309-311` uses `strlen(NMEA_GPTXT)` for GSA/GSV comparisons instead of their own prefix lengths.

---

## Python App State (`src/rocketlog/`)

All modules listed below are complete unless stated otherwise.

| Module | Status |
|--------|--------|
| `app.py` | Entry point. Creates `QApplication` + `MainWindow`. No GStreamer init yet. |
| `ui/style.py` | Full dark QSS theme. Green accent `#4dcc66`. Chip states: `ChipNeutral/Good/Caution/Warn`. Panels, telemetry rows, log view, scroll bars, buttons, inputs. |
| `ui/main_window.py` | `_SourceTab`: 4-panel grid (VIDEO, 3D SIM, TELEMETRY, LOG) + `LiveTabBar`. `MainWindow`: Rocket, CanSat, Playback, Settings tabs. `SessionRecorder` wired. `TelemetryWorker` running on a `QThread` — started in `_finish_startup`, stopped in `closeEvent`. `on_rocket_telemetry`, `on_cansat_telemetry`, `on_link_state`, `on_rocket_log`, `on_cansat_log` public slots. |
| `ui/video_panel.py` | `QPainter` widget. Scales `QImage` with aspect ratio. "NO VIDEO" placeholder. `set_frame(QImage)` / `set_recording(bool)` exist but are not yet wired to GStreamer. |
| `ui/telemetry_panel.py` | 11-row display: TIME UTC, LAT, LON, GPS ALT, BARO ALT, VELOCITY, SATELLITES, GPS FIX, TEMPERATURE, PRESSURE, BATTERY. `update_telemetry(Telemetry)` and `clear()`. |
| `ui/log_panel.py` | `QPlainTextEdit`-backed scrolling log. `append_line(line, level)` with colour coding. `append_log(level, msg, timestamp)`. Capped at 500 lines. |
| `ui/tab_bar.py` | `LiveTabBar`: per-tab strip with LINK chip, REC chip, Start/Stop Recording buttons. Signals: `start_recording`, `stop_recording`. `set_link_state_labelled(prefix, state)`, `set_recording(bool)`. `PlaybackTabBar`: Open button, path label, Rocket/CanSat radio buttons, speed combo, transport buttons, scrubber slider, timestamp label. |
| `ui/settings_tab.py` | Settings tab backed by `QSettings` (org=`rocketlog`, app=`rocketlog`). Sections: Serial/Connection (port combo + Refresh, baud, reconnect interval), Recording (output dir + Browse), Display (trail length, altitude scale). `read_str(key)` / `read_int(key)` / `write(key, val)` module-level helpers used by other modules. Emits `settings_changed` on Apply/Reset. |
| `ui/sim_panel.py` | **Next to rewrite.** Currently uses sphere-based projection — produces a bad viewing angle. Replace with a flat infinite-plane perspective grid (see "Next Steps" below). |
| `telemetry/types.py` | `Telemetry` TypedDict: `t_unix`, `alt_m`, `vel_mps`, `batt_v`, `temp_c`, `pressure_pa`, `lat`, `lon`, `gps_alt_m`, `gps_sats`, `gps_fix`. |
| `telemetry/protocol.py` | COBS framing, CRC32, packet encode/decode, all payload helpers. `ReceiverLog` dataclass. Matches firmware binary layout exactly. |
| `telemetry/events.py` | `TelemetryEvent`, `ReceiverLogEvent`, `AckEvent` frozen dataclasses. `TelemetryStreamEvent` union type. |
| `telemetry/reader.py` | `TelemetryReader`: opens serial port, performs TIME_SYNC handshake on construction, drains COBS frames, exposes `sample()` (blocking) and `receiver_log()` (non-blocking). |
| `telemetry/worker.py` | `TelemetryWorker(QObject)`: runs on a `QThread`, driven by an external `QTimer` calling `tick()`. Reads port/baud/reconnect-interval from `QSettings` on each connect attempt. Signals: `telemetry`, `receiver_log`, `state`, `error`, `info`. `reconnect()` slot forces an immediate retry (called on settings change). |
| `telemetry/__init__.py` | Empty placeholder. |
| `record/session.py` | `SessionRecorder`: multi-source JSONL + CSV recording + ZIP packaging to `.rocketlog`. |
| `record/manifest.py` | `Manifest` dataclass. |
| `util/graph.py` | Empty placeholder. |

`rocketlog.old` has working reference implementations of: telemetry protocol, serial reader, QThread worker, session recorder (JSONL + ZIP), GStreamer video pipeline, keyboard shortcuts.

---

## Next Steps (in order)

1. **Rewrite `ui/sim_panel.py`** — replace sphere projection with a flat infinite-plane perspective grid:
   - Camera at a fixed height above a flat ground plane, looking forward and slightly down.
   - Grid lines: evenly-spaced rails in Z (depth) and cross-lines in X (lateral), projected with pin-hole: `sx = f * x / z + cx`, `sy = f * y / z + cy`.
   - Keep: sky gradient background, trail, nadir crosshair, altitude stem + dot, "NO SIM DATA" label, colour constants.
   - Remove: all `_sphere_to_world`, `_project`, `_R`, `_CAM_H`, `_GRID_SPAN`, `_LAT_LINES`, `_LON_LINES`.
   - Add: `_CAM_HEIGHT`, `_GRID_DEPTH`, `_GRID_WIDTH`, `_GRID_COLS`, `_GRID_ROWS`.

2. **`video/` package** — GStreamer pipeline.

3. **Playback backend** — replay `.rocketlog` archives.

4. **Keyboard shortcuts** (see `.old` for reference).

5. **`util/graph.py`** — altitude/velocity graph widget.

---

## Code Style

| Language | Indent | Line width | Notes |
|----------|--------|------------|-------|
| C | Tabs (width 4) | 120 | LLVM clang-format, see `.clang-format` |
| Python | 4 spaces | — | PEP 8 |

### Python conventions (enforced across all `src/rocketlog/` files)

- Method separators: single-line `# ---------------------------------------- #`
- Section headers (module-level or between major class blocks): three lines:
  ```python
  # ---------------------------------------- #
  # Title                                     #
  # ---------------------------------------- #
  ```
- All methods have concise docstrings.
- Comments are concise.
- No emojis.

LSP: clangd configured for ESP32-S3 in `.clangd`. `compile_commands.json` is a symlink managed by `just clangd-use-*`.

---

## Development Commands

```bash
just app-run              # Install + run Python app (uses .venv)
just receiver-build       # Build receiver firmware
just receiver-burn        # Build + flash receiver
just receiver-monitor     # Open idf.py serial monitor
just transmitter-build    # Build transmitter firmware
just transmitter-burn     # Build + flash transmitter
just clangd-use-receiver  # Point compile_commands.json at receiver build
just clangd-use-transmitter  # Point compile_commands.json at transmitter build
```

Python venv: `.venv/` — create with `python3 -m venv .venv --system-site-packages` (`--system-site-packages` is required for GStreamer `gi` bindings).

ESP-IDF: v5.5.2 at `/opt/esp-idf`. Serial ports in `firmware/flash.toml`.

---

## Deployment Target: Raspberry Pi 3 (Cyberdeck)

The app runs on PiOS (Debian Bookworm armhf/arm64). Key considerations:

- **PySide6**: not in Debian repos for armhf; install via pip or use the `pyside6` package from PiOS if available. Prefer system Qt6 + PyQt6 as fallback if PySide6 proves painful on Pi.
- **GStreamer**: install from apt, not pip. Required packages: `gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad python3-gst-1.0 python3-gi`. The `gi` module comes from `python3-gi` and cannot be pip-installed — this is why the venv needs `--system-site-packages`.
- **Serial**: user must be in the `dialout` group (`sudo usermod -aG dialout $USER`).
- **Performance**: Pi 3 is ARMv8 with 1 GB RAM. Avoid heavy Python-side processing on the UI thread. GStreamer pipelines should do the heavy lifting for video.
- **Display**: assumes a connected display (HDMI or DSI). No headless mode needed.
