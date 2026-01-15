#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class FlashConfig:
    port: str | None
    baud: int | None


# ---------------------------------------- #


def load_config(path: Path, project: str) -> FlashConfig:
    data = tomllib.loads(path.read_text(encoding="utf-8"))

    table = data.get(project)
    if not isinstance(table, dict):
        raise ValueError(f"Missing [{project}] table in {path}")

    port = table.get("port")
    baud = table.get("baud")

    if port is not None and not isinstance(port, str):
        raise ValueError(f"[{project}].port must be a string")
    if baud is not None and not isinstance(baud, int):
        raise ValueError(f"[{project}].baud must be an integer")

    return FlashConfig(port=port, baud=baud)


# ---------------------------------------- #


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Read firmware/flash.toml")
    parser.add_argument("project", choices=["receiver", "transmitter"])
    parser.add_argument("key", choices=["port", "baud"])
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "flash.toml"),
        help="Path to flash.toml (default: firmware/flash.toml)",
    )

    args = parser.parse_args(argv)

    cfg = load_config(Path(args.config), args.project)

    value = getattr(cfg, args.key)
    if value is None:
        return 1

    sys.stdout.write(str(value))
    return 0


# ---------------------------------------- #

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
