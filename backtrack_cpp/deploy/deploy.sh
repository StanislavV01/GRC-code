#!/usr/bin/env bash
# Build & deploy the backtrack core onto a Raspberry Pi (CM4).
#
# The code is pure stdlib C++17, so the simplest reliable path is: sync the
# sources and build natively on the Pi (it has g++). No cross-toolchain needed.
#
# Usage:
#   PI=pi@raspberrypi.local ./deploy/deploy.sh            # sync + build + run once
#   PI=pi@raspberrypi.local ./deploy/deploy.sh --service  # + demo systemd unit
#   PI=pi@raspberrypi.local ./deploy/deploy.sh --vehicle  # + can0 + vehicle units
#
# First time on a fresh Pi run ./deploy/setup_cm4.sh ON THE PI (overlays for
# I2C/SPI/MCP2515/UART), reboot, then deploy. See docs/HARDWARE_BRINGUP.md.
#
# Requires: ssh access to the Pi, and g++ on the Pi (sudo apt install -y g++).
set -euo pipefail

PI="${PI:-pi@raspberrypi.local}"
DEST="${DEST:-backtrack_cpp}"
SRC_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "[1/3] sync sources -> ${PI}:${DEST}"
rsync -az --delete --exclude 'build/' --exclude 'data/' \
    "${SRC_DIR}/" "${PI}:${DEST}/"

echo "[2/3] build on the Pi"
ssh "${PI}" "cd ${DEST} && CXX=g++ ./build.sh"

echo "[3/3] smoke run on the Pi (writes data/backtrack.log)"
ssh "${PI}" "cd ${DEST} && ./build/run_demo --log-file data/backtrack.log && tail -n 5 data/backtrack.log"

if [[ "${1:-}" == "--service" ]]; then
    echo "[svc] installing demo systemd unit"
    scp "${SRC_DIR}/deploy/backtrack.service" "${PI}:/tmp/backtrack.service"
    ssh "${PI}" "sudo mv /tmp/backtrack.service /etc/systemd/system/ \
        && sudo systemctl daemon-reload \
        && sudo systemctl enable --now backtrack \
        && echo 'service installed; logs: journalctl -u backtrack -f'"
fi

if [[ "${1:-}" == "--vehicle" ]]; then
    echo "[svc] installing can0 + vehicle systemd units"
    scp "${SRC_DIR}/deploy/can0.service" "${SRC_DIR}/deploy/vehicle.service" \
        "${PI}:/tmp/"
    ssh "${PI}" "sudo mv /tmp/can0.service /tmp/vehicle.service /etc/systemd/system/ \
        && sudo systemctl daemon-reload \
        && sudo systemctl enable --now can0 \
        && sudo systemctl enable --now vehicle \
        && echo 'vehicle service up; logs: journalctl -u vehicle -f'"
fi

echo "[done]"
