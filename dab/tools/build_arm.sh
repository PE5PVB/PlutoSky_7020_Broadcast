#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Cross build of the DAB software for the Pluto's ARM (Cortex-A9, NEON) with the buildroot toolchain of the devkit.
# Output in build-arm/. Nothing is copied to a board.
set -e
DK="${DK:-$HOME/fishball7020-fpga-devkit}"
GCC="$DK/firmware/src/buildroot/output/host/bin/arm-linux-gnueabihf-gcc"
[ -x "$GCC" ] || { echo "cross compiler not found: $GCC"; exit 1; }
cd "$(dirname "$0")/.."
make -s BUILD=build-arm CC="$GCC" CFLAGS="-O3 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -ffast-math -Wall -Wextra -std=gnu11" all
ls -la build-arm/enc_file build-arm/dab_tx_file
