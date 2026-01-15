# SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0
from __future__ import annotations

try:
    import pytest
    from pytest_embedded import Dut
    from pytest_embedded_idf.utils import idf_parametrize
except ModuleNotFoundError:
    # This test is vendored from ESP-IDF examples; it's optional here.
    pytest = None  # type: ignore
    Dut = object  # type: ignore

    def idf_parametrize(*args, **kwargs):  # type: ignore
        def decorator(fn):
            return fn

        return decorator


if pytest is not None:

    @pytest.mark.generic
    @idf_parametrize(
        "target",
        [
            "esp32",
            "esp32s2",
            "esp32s3",
            "esp32c3",
            "esp32c5",
            "esp32c6",
            "esp32h2",
            "esp32p4",
        ],
        indirect=["target"],
    )
    def test_led_strip_example(dut) -> None:
        dut.expect_exact("example: Create RMT TX channel")
        dut.expect_exact("example: Install led strip encoder")
        dut.expect_exact("example: Enable RMT TX channel")
        dut.expect_exact("example: Start LED rainbow chase")
