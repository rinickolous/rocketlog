# RocketLog (v0, GStreamer + PySide6)

A minimal prototype “ground station” desktop app written in Python that:

- Shows a live **webcam** feed (GStreamer `v4l2src`)
- Displays **simulated telemetry** in real time
- Records **MP4 video + telemetry JSONL**
- Packages both into a single archive file: **`.rocketlog`** (a ZIP container)

This is a **hardware-free** scaffold intended for early UI and recording workflow development on Linux.

---

## Features (v0)

- Live preview from `/dev/video*` using GStreamer
- GUI using PySide6 (Qt)
- Simulated telemetry at 10 Hz
- One-click recording:
  - `video.mp4` (H.264 via `x264enc`, MP4 mux)
  - `telemetry.jsonl` (timestamped JSON per line)
  - `manifest.json` (pipeline + metadata)
- Single packaged file output: `recording_YYYYMMDDTHHMMSSZ.rocketlog`

---

## Project Layout

This v0 is typically a single-file script:

- `main.py` — the application
- `recordings/` — output directory created at runtime

A `.rocketlog` archive contains:

- `video.mp4`
- `telemetry.jsonl`
- `manifest.json`

---

## Requirements

### OS / Runtime

- Linux (tested on Arch Linux)
- Python 3.10+ (3.11/3.12 recommended)

### System Packages (Arch Linux)

Install the GStreamer stack, codec plugins, GI bindings, and tools:

```bash
sudo pacman -S --needed \
  gstreamer \
  gst-plugins-base \
  gst-plugins-good \
  gst-plugins-bad \
  gst-plugins-ugly \
  gst-libav \
  python-gobject \
  gobject-introspection \
  gst-devtools
```
