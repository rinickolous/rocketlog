from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class FirmwareFlashConfig:
    port: str | None = None
    baud: int | None = None


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


# ---------------------------------------- #


def load_firmware_flash_config(
    project: str, path: Path | None = None
) -> FirmwareFlashConfig:
    if path is None:
        path = _repo_root() / "firmware" / "flash.toml"

    try:
        import tomllib
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("tomllib unavailable; need Python 3.11+ or tomli") from exc

    try:
        data: Any = tomllib.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return FirmwareFlashConfig()

    table = data.get(project)
    if not isinstance(table, dict):
        return FirmwareFlashConfig()

    port = table.get("port")
    baud = table.get("baud")

    if port is not None and not isinstance(port, str):
        port = None
    if baud is not None and not isinstance(baud, int):
        baud = None

    return FirmwareFlashConfig(port=port, baud=baud)
