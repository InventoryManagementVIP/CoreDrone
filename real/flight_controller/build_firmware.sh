#!/usr/bin/env bash
# Build ArduCopter with DDS for the Pixhawk 6C. Run through `just fc-firmware` on an x86-64 host.
#
# The board's hwdef does not turn DDS on, so it needs --enable-DDS at configure time. Uses the
# ArduPilot tree pinned in the dev image, copied so that the image's SITL build stays untouched.
# Output: runs/firmware/arducopter.apj
set -euo pipefail

BOARD="${1:-Pixhawk6C}"
TOOLCHAIN=gcc-arm-none-eabi-10-2020-q4-major # from ArduPilot's own firmware server
WORK="${TMPDIR:-/tmp}/coredrone-firmware"
OUT="${INV_ROOT:-/ws}/runs/firmware"

mkdir -p "$WORK" "$OUT"
if [ ! -d "$WORK/$TOOLCHAIN" ]; then
  curl -fsSL "https://firmware.ardupilot.org/Tools/STM32-tools/$TOOLCHAIN-x86_64-linux.tar.bz2" | tar xj -C "$WORK"
fi
export PATH="$WORK/$TOOLCHAIN/bin:/opt/underlay/src/Micro-XRCE-DDS-Gen/scripts:$PATH"

rm -rf "$WORK/ardupilot"
cp -r /opt/underlay/src/ardupilot "$WORK/ardupilot"
cd "$WORK/ardupilot"
git config --global --add safe.directory '*'
git submodule update --init --recursive --depth 1 modules/ChibiOS modules/CrashDebug

python3 ./waf configure --board "$BOARD" --enable-DDS --no-submodule-update
python3 ./waf copter -j"$(nproc)"
cp "build/$BOARD/bin/arducopter.apj" "$OUT/"
echo "Built ArduPilot $(git rev-parse --short HEAD). Flash runs/firmware/arducopter.apj as custom firmware."
