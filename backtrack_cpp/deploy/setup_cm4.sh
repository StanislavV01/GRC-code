#!/usr/bin/env bash
# One-time CM4 bring-up for the backtrack vehicle stack. Run ON THE PI as a
# user with sudo. Idempotent: safe to re-run.
#
#   ./deploy/setup_cm4.sh          # configure overlays + services, then reboot
#
# What it does:
#   1. /boot config: I2C on (GY-9250), SPI + MCP2515 overlay (Waveshare RS485
#      CAN HAT -> can0), primary UART free for RS485.
#   2. Removes the serial login console from cmdline.txt (it would eat the
#      operator frames on /dev/serial0).
#   3. Installs can0.service (ip link ... bitrate 500000 at boot).
#
# NOTE: the Waveshare RS485 CAN HAT ships with a 12 MHz crystal; some clones
# use 8 MHz. Check the crystal marking and adjust OSC below if CAN stays silent.
set -euo pipefail

OSC="${OSC:-12000000}"          # MCP2515 crystal: 12000000 or 8000000
CAN_INT_GPIO="${CAN_INT_GPIO:-25}"

# Raspberry Pi OS bookworm keeps config.txt in /boot/firmware.
CONFIG=/boot/firmware/config.txt
CMDLINE=/boot/firmware/cmdline.txt
[[ -f "$CONFIG" ]] || CONFIG=/boot/config.txt
[[ -f "$CMDLINE" ]] || CMDLINE=/boot/cmdline.txt

echo "[1/3] $CONFIG: i2c + spi + mcp2515 + uart"
add_line() {
    grep -qF "$1" "$CONFIG" || echo "$1" | sudo tee -a "$CONFIG" >/dev/null
}
add_line "dtparam=i2c_arm=on"
add_line "dtparam=spi=on"
add_line "dtoverlay=mcp2515-can0,oscillator=${OSC},interrupt=${CAN_INT_GPIO},spimaxfrequency=2000000"
add_line "enable_uart=1"

echo "[2/3] $CMDLINE: remove serial login console"
sudo sed -i 's/console=serial0,115200 //; s/console=ttyAMA0,115200 //' "$CMDLINE"

echo "[3/3] install can0.service (500 kbit/s at boot)"
sudo cp "$(dirname "$0")/can0.service" /etc/systemd/system/can0.service
sudo systemctl daemon-reload
sudo systemctl enable can0.service

sudo usermod -aG i2c,dialout "$USER" || true

echo
echo "Done. Reboot, then verify:"
echo "  i2cdetect -y 1        # expect 68 (GY-9250)"
echo "  ip -details link show can0   # expect 'state UP ... bitrate 500000'"
echo "  candump can0          # expect 0x0901/0x0902 status frames from VESCs"
