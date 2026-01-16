# ---------------------------------------- #
#  Rocketlog development commands           #
# ---------------------------------------- #

set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

# ---------------------------------------- #
#  Config file for serial port / baud.     #
# ---------------------------------------- #
flash_config := "firmware/flash.toml"
flash_cfg := "python firmware/flash_config.py"

# ---------------------------------------- #
#  Python app venv                          #
# ---------------------------------------- #
venv_python := ".venv/bin/python"

# ---------------------------------------- #
#  ESP-IDF (run in an isolated subshell)   #
# ---------------------------------------- #
idf_export := "/opt/esp-idf/export.sh"

# ---------------------------------------- #
#  Clangd (Neovim)                         #
# ---------------------------------------- #

clangd-use-receiver:
    bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 || true; ln -sf firmware/receiver/build/compile_commands.json compile_commands.json'

clangd-use-transmitter:
    bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 || true; ln -sf firmware/transmitter/build/compile_commands.json compile_commands.json'

# ---------------------------------------- #
#  App                                     #
# ---------------------------------------- #

app-build:
    @test -x {{venv_python}} || { \
        echo "error: missing venv python: {{venv_python}}" >&2; \
        echo "hint: create it with: python3 -m venv .venv --system-site-packages" >&2; \
        exit 127; \
    }
    {{venv_python}} -m pip install -e .

app-run: app-build
    {{venv_python}} -m rocketlog

# ---------------------------------------- #
#  ESP-IDF env check                       #
# ---------------------------------------- #

idf-check:
    @test -f "{{idf_export}}" || { \
        echo "error: ESP-IDF export script not found: {{idf_export}}" >&2; \
        echo "hint: edit idf_export near top of justfile" >&2; \
        exit 127; \
    }
    @bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 && command -v idf.py >/dev/null 2>&1' || { \
        echo "error: idf.py not available after sourcing {{idf_export}}" >&2; \
        exit 127; \
    }

# ---------------------------------------- #
#  Receiver (ESP-IDF)                      #
# ---------------------------------------- #

receiver-build: idf-check
    bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 && idf.py -C firmware/receiver build'
    just clangd-use-receiver

receiver-flash: idf-check
    bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 && \
        idf.py -C firmware/receiver \
            -p "{{ `python firmware/flash_config.py receiver port --config firmware/flash.toml` }}" \
            -b "{{ `python firmware/flash_config.py receiver baud --config firmware/flash.toml` }}" \
            flash'

receiver-monitor: idf-check
    bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 && \
        idf.py -C firmware/receiver \
            -p "{{ `python firmware/flash_config.py receiver port --config firmware/flash.toml` }}" \
            -b "{{ `python firmware/flash_config.py receiver baud --config firmware/flash.toml` }}" \
            monitor'

receiver-burn: receiver-build receiver-flash

# ---------------------------------------- #
#  Transmitter (ESP-IDF)                   #
# ---------------------------------------- #

transmitter-build: idf-check
    bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 && idf.py -C firmware/transmitter build'
    just clangd-use-transmitter

transmitter-flash: idf-check
    bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 && \
        idf.py -C firmware/transmitter \
            -p "{{ `python firmware/flash_config.py transmitter port --config firmware/flash.toml` }}" \
            -b "{{ `python firmware/flash_config.py transmitter baud --config firmware/flash.toml` }}" \
            flash'

transmitter-monitor: idf-check
    bash -lc 'source /opt/esp-idf/export.sh >/dev/null 2>&1 && \
        idf.py -C firmware/transmitter \
            -p "{{ `python firmware/flash_config.py transmitter port --config firmware/flash.toml` }}" \
            -b "{{ `python firmware/flash_config.py transmitter baud --config firmware/flash.toml` }}" \
            monitor'

transmitter-burn: transmitter-build transmitter-flash
