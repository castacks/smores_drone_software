#!/usr/bin/env bash
epson_tty=$1

if [ -z "$epson_tty" ]; then
    exit 1
fi

echo 1 > /sys/bus/usb-serial/devices/${epson_tty}/latency_timer
