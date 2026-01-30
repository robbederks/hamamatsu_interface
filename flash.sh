#!/bin/bash -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
FW_DIR="${DIR}/firmware/hamamatsu_interface"

cd "${FW_DIR}"
pio run --target upload --environment teensy41


