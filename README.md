# RocketLog Ground Station

A modular Python ground-station application for small rocketry and telemetry experiments.

RocketLog provides:

- Live **video preview** via GStreamer
- Live **telemetry display** (currently simulated)
- Session **recording** (video + telemetry packaged together)
- A **modern “glass cockpit” UI** (Qt / PySide6)
- Designed to run both on a **desktop Linux machine** and a **headless / kiosk Raspberry Pi**
- Clean architecture ready for **physical hardware** (LEDs, 7-segment displays, buttons)

This repository is structured as a proper Python package and is intended to be installable directly from source.

---

## Current status

- UI: Qt Widgets (PySide6)
- Video: GStreamer via `gi.repository`
- Telemetry: simulated source (pluggable later)
- Recording: session-based, `.rocketlog` archive
- Keyboard shortcuts: implemented
- Hardware IO (LEDs / 7-segment): **planned**, not yet wired in code

---

## Runtime requirements

### Operating system

- Linux (tested on Arch Linux and Raspberry Pi OS)
- Wayland or X11 (kiosk mode supported)

### System packages (GStreamer + GI)

#### Arch Linux

```bash
sudo pacman -S --needed \
  gstreamer \
  gst-plugins-base \
  gst-plugins-good \
  gst-plugins-bad \
  gst-plugins-ugly \
  gst-libav \
  python-gobject \
  gobject-introspection
```

#### Debian / Raspberry Pi OS

```bash
sudo apt update
sudo apt install -y \
  python3-venv python3-pip \
  python3-gi python3-gst-1.0 gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav
```

---

## First Time Setup

```bash
git clone https://github.com/rinickolous/rocketlog
cd rocketlog

# Because GI bindings are installed system-wide, the venv must see system packages:
python3 -m venv .venv --system-site-packages

# Activate venv:
source .venv/bin/activate

# Install Python dependencies:
pip install --upgrade pip
pip install -e .
```

## Launching RocketLog

Works after `pip install -e .`

```bash
rocketlog
```

Thin launcher:

```bash
python main.py
```

---

## Keyboard Shortcuts

| Key      | Action               |
| -------- | -------------------- |
| `R`      | Start recording      |
| `S`      | Stop recording       |
| `Space`  | Toggle record / stop |
| `F`      | Toggle fullscreen    |
| `Ctrl+Q` | Quit application     |
