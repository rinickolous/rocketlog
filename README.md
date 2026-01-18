# RocketLog

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

---

## Runtime requirements

### Supported OS

- Linux (tested on Arch Linux)
- Wayland or X11

### Python

- Python 3.10+

### Python dependencies

Installed via `pip install -e .`:

- `PySide6` (Qt UI)

Also required at runtime and must be installed via OS packages (not pip):

- `python-gobject` / `python3-gi` (provides `gi`)
- GStreamer 1.0 runtime + plugins (see distro-specific package lists below)

If you see an error like `ModuleNotFoundError: No module named 'gi'`, you are missing the OS-level GI/GStreamer packages.

### System dependencies (GStreamer + GI bindings)

RocketLog uses GStreamer via Python GObject Introspection (`gi.repository`). Those bindings are typically installed via your OS package manager (not pip).

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

#### Ubuntu

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

#### Fedora

Some GStreamer plugin packages (notably `-ugly` and `-libav`) may require RPM Fusion.

```bash
# Enable RPM Fusion (recommended for full codec/plugin coverage)
sudo dnf install -y \
  https://download1.rpmfusion.org/free/fedora/rpmfusion-free-release-$(rpm -E %fedora).noarch.rpm \
  https://download1.rpmfusion.org/nonfree/fedora/rpmfusion-nonfree-release-$(rpm -E %fedora).noarch.rpm

sudo dnf install -y \
  python3-pip python3-virtualenv \
  python3-gobject \
  gstreamer1 \
  gstreamer1-plugins-base \
  gstreamer1-plugins-good \
  gstreamer1-plugins-bad-free \
  gstreamer1-plugins-ugly \
  gstreamer1-libav
```

---

## Install

```bash
git clone https://github.com/rinickolous/rocketlog
cd rocketlog

# GI bindings are installed system-wide, so the venv must see system packages:
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

python -m pip install --upgrade pip
python -m pip install -e .
```

## Launch

After install:

```bash
rocketlog
```

Or run as a module:

```bash
python -m rocketlog
```

---

## Firmware (ESP-IDF)

This repo also contains two ESP-IDF projects:

- `firmware/receiver`: telemetry receiver/simulator (ESP32-S3)
- `firmware/transmitter`: minimal transmitter stub (ESP32-S3)

### Prerequisites

- ESP-IDF installed (tested with ESP-IDF v5.5.2)
- `just` installed for convenient one-liners (optional but recommended)

#### Install `just`

Arch Linux:

```bash
sudo pacman -S --needed just
```

Ubuntu / Debian / Raspberry Pi OS:

```bash
sudo apt update
sudo apt install -y just
```

Fedora:

```bash
sudo dnf install -y just
```

### Build / flash (via just)

#### Troubleshooting

If `just receiver-flash` fails with a serial/monitor error, check the `baud` in `firmware/flash.toml`.
ESP-IDF defaults to `115200` for monitor output unless you changed the project config.

The Python ground-station telemetry reader also defaults to the `receiver` settings in `firmware/flash.toml`.

All commands run from the repo root.

```bash
just app-build
just app-run

just receiver-build
just receiver-flash
just receiver-burn

just transmitter-build
just transmitter-flash
just transmitter-burn
```

### Flash configuration

Serial ports and baud rates are configured in `firmware/flash.toml`.

You should edit it once for your machine (example):

```toml
[receiver]
port = "/dev/ttyACM0"
baud = 115200

[transmitter]
port = "/dev/ttyACM1"
baud = 115200
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
