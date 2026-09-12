#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
# ATK-DLRK3588 UART3 only. Leaves UART2 and its console untouched.
set -euo pipefail
mode=${1:---rust}
case "$mode" in --rust|--restore) ;; *) echo "Usage: $0 [--rust|--restore]" >&2; exit 2;; esac
[[ $EUID -eq 0 ]] || { echo 'Run as root on the development board.' >&2; exit 1; }
model=$(tr -d '\0' < /proc/device-tree/model)
[[ $model == *ATK-DLRK3588* ]] || { echo 'This helper is only for ATK-DLRK3588.' >&2; exit 1; }
devname=feb60000.serial
device=/sys/bus/platform/devices/$devname
[[ -d $device ]] || { echo 'UART3 is absent; enable uart3m2_xfer in the DTB first.' >&2; exit 1; }
cmdline=$(cat /proc/cmdline)
[[ ! $cmdline =~ (^|[[:space:]])console=tty(S3|RU0)(,|[[:space:]]|$) ]] || {
	echo 'Refusing to rebind a console UART.' >&2; exit 1;
}
active=$(cat /sys/class/tty/console/active)
[[ ! $active =~ (^|[[:space:]])tty(S3|RU0)($|[[:space:]]) ]] || {
	echo 'Refusing to rebind an active console UART.' >&2; exit 1;
}
command -v fuser >/dev/null || { echo 'Install psmisc to check for open UART files.' >&2; exit 1; }
for tty in /dev/ttyS3 /dev/ttyRU0; do
	if [[ -e $tty ]] && fuser -s "$tty"; then
		echo "$tty is in use; close its applications first." >&2
		exit 1
	fi
done
current=
if [[ -L $device/driver ]]; then current=$(basename "$(readlink "$device/driver")"); fi
if [[ $mode == --restore ]]; then
	[[ $current == rust_dw_uart ]] || {
		echo 'UART3 is not bound to rust_dw_uart.' >&2; exit 1;
	}
	printf '%s' "$devname" > /sys/bus/platform/drivers/rust_dw_uart/unbind
	printf '\n' > "$device/driver_override"
	printf '%s' "$devname" > /sys/bus/platform/drivers_probe
	[[ -c /dev/ttyS3 ]] || {
		echo 'Default driver did not create ttyS3; inspect dmesg.' >&2; exit 1;
	}
	echo 'UART3 restored to the default driver: /dev/ttyS3'
	exit 0
fi
if [[ $current == rust_dw_uart ]]; then echo 'UART3 is already using /dev/ttyRU0.'; exit 0; fi
override=$(cat "$device/driver_override")
[[ -z $override || $override == '(null)' ]] || {
	echo 'An existing driver_override is set; refusing to replace it.' >&2; exit 1;
}
modprobe rust_dw_uart
rollback() {
	if [[ -L $device/driver ]] &&
		[[ $(basename "$(readlink "$device/driver")") == rust_dw_uart ]]; then
		printf '%s' "$devname" > /sys/bus/platform/drivers/rust_dw_uart/unbind
	fi
	printf '\n' > "$device/driver_override"
	if [[ ! -L $device/driver ]]; then
		if [[ -n $current ]]; then
			printf '%s' "$devname" > "/sys/bus/platform/drivers/$current/bind"
		else
			printf '%s' "$devname" > /sys/bus/platform/drivers_probe
		fi
	fi
}
trap rollback ERR
printf '%s' rust_dw_uart > "$device/driver_override"
if [[ -n $current ]]; then printf '%s' "$devname" > "/sys/bus/platform/drivers/$current/unbind"; fi
printf '%s' "$devname" > /sys/bus/platform/drivers/rust_dw_uart/bind
[[ -c /dev/ttyRU0 ]]
trap - ERR
echo 'UART3 is using the Rust controller: /dev/ttyRU0'
