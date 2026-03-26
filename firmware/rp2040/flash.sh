#!/usr/bin/env bash
# flash.sh — Reset RP2040 into BOOTSEL mode, mount, flash .uf2, unmount.
# Usage: flash.sh <path-to-uf2>
#
# Prerequisites (one-time, requires sudo):
#   sudo sh -c 'echo "SUBSYSTEM==\"gpio\", KERNEL==\"gpiochip[0-9]*\", MODE=\"0660\", GROUP=\"dialout\"" > /etc/udev/rules.d/99-gpio.rules'
#   sudo udevadm control --reload-rules && sudo udevadm trigger
#   sudo usermod -aG dialout $USER
#   # Log out and back in for group change to take effect
set -euo pipefail

UF2_FILE="${1:?Usage: $0 <path-to-.uf2>}"

if [ ! -f "$UF2_FILE" ]; then
    echo "Error: file not found: $UF2_FILE" >&2
    exit 1
fi

RP2_LABEL="/dev/disk/by-label/RPI-RP2"

if [ ! -e "$RP2_LABEL" ]; then
    # Toggle GPIOs to reset RP2040 into BOOTSEL mode
    echo "Resetting RP2040 into BOOTSEL mode..."
    gpioset gpiochip0 17=1
    gpioset gpiochip0 7=1
    sleep 1
    gpioset gpiochip0 17=0
    gpioset gpiochip0 7=0
else
    echo "RP2040 already in BOOTSEL mode."
fi

# Wait for the RP2040 USB mass storage device to appear
echo "Waiting for RP2040 USB device..."
timeout=10
while [ $timeout -gt 0 ]; do
    if [ -e "$RP2_LABEL" ]; then
        break
    fi
    sleep 1
    timeout=$((timeout - 1))
done

if [ ! -e "$RP2_LABEL" ]; then
    echo "Error: timed out waiting for RP2040 USB device" >&2
    exit 1
fi

# Resolve symlink to actual block device
block_dev=$(readlink -f "$RP2_LABEL")
echo "Found RP2040 at $block_dev"

# Mount via udisksctl (no sudo required)
mount_output=$(udisksctl mount -b "$block_dev" 2>&1) || true
mount_point=$(echo "$mount_output" | grep -oP '(?:at |at `)\K/[^\x27\s]+')

if [ -z "$mount_point" ]; then
    echo "Error: failed to mount: $mount_output" >&2
    exit 1
fi

echo "Mounted at $mount_point"

if [ ! -f "$mount_point/INFO_UF2.TXT" ]; then
    echo "Unable to find expected file INFO_UF2.TXT. Exiting..." 2>&1
    exit 1
fi

# Copy firmware image
cp "$UF2_FILE" "$mount_point/"
sync

echo "Firmware flashed. RP2040 will reboot automatically."

# Unmount — the device will disappear after reboot anyway, but clean up
udisksctl unmount -b "$block_dev" 2>/dev/null || true
