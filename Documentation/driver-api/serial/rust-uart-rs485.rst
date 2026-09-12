.. SPDX-License-Identifier: GPL-2.0-only

Rust UART and character-device development on ATK-DLRK3588
=========================================================

Architecture and scope
----------------------

``rust_dw_uart`` is a UART **controller** driver. Its register accesses,
interrupt dispatch, FIFO service, baud divisor/framing changes, and break
control are implemented in Rust. It registers one ``ttyRU0`` line through
the existing C serial core and TTY subsystem. It does not call the C 8250
controller's interrupt or register-access implementation.

``rust_chardev`` is an independent miscellaneous character-device driver.
It provides a bounded FIFO for learning and regression testing; it is not
a wrapper around ``ttyRU0`` and does not access the RS485 hardware.

The new C code consists only of wrappers for existing serial-core inline
functions and macros that bindgen cannot expose directly. These live in
``rust/helpers/serial.c``. Rust/C pointer and lifetime handling is confined
to ``rust/kernel/serial.rs`` and the existing kernel abstractions.

This is a local development implementation, not a claim of upstream
acceptance or production qualification. Current controller support is
RK3588, 32-bit MMIO with a register shift of two, a 24 MHz baud clock,
interrupt-driven I/O, and one non-console port. Console integration, DMA,
modem flow control, automatic runtime power management, and system suspend
are outside its current scope. Do not suspend the system with this driver
bound. The baud clock is enabled and rate-protected while bound.
Fractional-divisor width and FIFO depth are discovered from the controller.
Divisors are rounded with fractional carry, and baud rates with more than
two percent quantization error are rejected by restoring the prior termios.
The application checks the effective settings after configuring the TTY.

Layout
------

* ``drivers/tty/serial/rust_dw_uart.rs``: UART platform/controller driver.
* ``drivers/tty/serial/rust_dw_uart/config.rs``: register encoding.
* ``rust/kernel/serial.rs``: serial-core registration and locked port API.
* ``rust/helpers/serial.c``: inline/macro compatibility wrappers.
* ``drivers/misc/rust_chardev.rs``: character-device operations.
* ``drivers/misc/rust_chardev/ring.rs``: bounded FIFO indices.
* ``tools/testing/selftests/rust_uart_rs485/``: Rust test application,
  peer, host tests, and kselftest entry point.
* ``arch/arm64/boot/dts/rockchip/rk3588-atk-dlrk3588.dts``: UART3 M2 wiring.

Kernel code uses Kconfig and Kbuild, not Cargo. Cargo is used only for
userspace programs and host-side tests. No private ioctl ABI is required.

Hardware
--------

On ATK-DLRK3588B, power off before moving both JP6 jumpers to RS485:
short 1--3 and 2--4. UART3 TX (GPIO4_A5) connects to RS485_RX; UART3 RX
(GPIO4_A6) connects to RS485_TX. JP3/UART2 is the debug console and is
unrelated to these jumpers. For a two-device test, connect RS485 A to A
and B to B with an appropriate signal-reference arrangement.

The board's SP3485EN circuit automatically controls transmit direction.
The software uses ordinary 8N1 UART I/O with hardware flow control off.
There is no additional RTS/DE ioctl or GPIO operation. The populated
120-ohm board termination must be considered when terminating the bus;
do not add another resistor blindly in parallel at the same endpoint.

See the manufacturer's hardware manual, sections 3.28 and 3.29:
https://wiki.alientek.com/docs/category/atk-dlrk3588-1/

Build
-----

Enable ``CONFIG_RUST=y``, ``CONFIG_SERIAL_CORE=y``,
``CONFIG_SERIAL_RUST_DW=m``, ``CONFIG_RUST_CHARDEV=m`` and module unloading.
Follow ``Documentation/rust/quick-start.rst`` for the Rust, Clang, libclang,
and bindgen requirements. Check ``make ARCH=arm64 LLVM=1 rustavailable``
with the same compiler settings that will be used for the kernel.

Use a separate output directory. Build a complete kernel and its modules::

    make O=/path/to/build ARCH=arm64 LLVM=1 -j16 \
        Image modules rockchip/rk3588-atk-dlrk3588.dtb

``modules_prepare`` is not a replacement for this build. Deploy Image,
DTB and modules from the same output directory. A Rust module built here
cannot be inserted into the older running kernel which lacks Rust support.

Build the application::

    cd tools/testing/selftests/rust_uart_rs485
    cargo test --locked
    cargo clippy --locked --all-targets -- -D warnings
    cargo build --locked --release --target aarch64-unknown-linux-gnu

Set ``CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER`` to the board's cross
linker when building on x86. ``Cargo.lock`` pins userspace dependencies.
The application's MSRV is independent of the kernel's toolchain minimum.

On the tested ATK board, the capability register reports a 64-byte FIFO
and zero DLF bits. With this driver's fixed 24 MHz clock, internal loopback
has been verified at 9600, 19200, 38400, 57600, 115200 and 1500000 baud.
Rates such as 230400 require CRU clock-rate adaptation; the current driver
rejects excessive divisor error instead of silently running at a wrong rate.

Explicit controller binding
---------------------------

The normal DT compatible remains ``rockchip,rk3588-uart`` with its existing
fallback. Firmware describes hardware, not the language of its driver.
No driver-specific compatible string is introduced.

There is intentionally no automatic OF match table in the Rust driver.
After deploying the new kernel, load and bind it explicitly::

    modprobe rust_dw_uart
    tools/testing/selftests/rust_uart_rs485/bind-uart3.sh --rust

The helper is specific to ``feb60000.serial`` (UART3). It rejects a console
UART, checks for open device files using ``fuser`` (psmisc), preserves an
existing override by refusing to replace it, and rolls back a failed bind.
The resulting node is ``/dev/ttyRU0``; ``/dev/ttyS3`` belongs to the former
8250 driver and must not be used concurrently.

After closing the UART application, restore the default controller::

    tools/testing/selftests/rust_uart_rs485/bind-uart3.sh --restore

Controller tests
----------------

First test the UART's internal hardware loopback, without an external peer::

    rs485-test loopback --device /dev/ttyRU0 --count 1000 --size 1024

The test enables ``TIOCM_LOOP`` and restores its prior state on exit.
This exercises the UART controller but does not validate the transceiver,
JP6 jumpers, cable, or differential bus.

For RS485, first run the peer on a second Linux machine with a USB-RS485
adapter (build the same application for that machine)::

    rs485-test peer --device /dev/ttyUSB0 --baud 115200 --count 1000

Then run the board-side test::

    rs485-test uart --device /dev/ttyRU0 --baud 115200 --count 1000 --size 1024

Each frame contains a magic value, sequence number, length, payload and
IEEE CRC-32. The peer checks the complete request before replying. Every
response must be byte-for-byte identical. This is a test protocol, not
Modbus RTU. Success of write() alone never implies on-wire completion.

Both sides leave at least four character times or 2 ms, whichever is longer,
before reversing direction. Immediate byte delivery by the interrupt-driven
controller does not mean the other transceiver has released the bus. At the
end of the run the peer drains output and allows one final frame's wire time
before restoring termios: USB adapters such as CH340 may report an empty
kernel queue while their hardware still has bytes to transmit.

The application handles partial read/write, EINTR, EAGAIN, and finite
deadlines. Device settings are restored on normal/error exit. A peer must
be restarted for a new test run because sequences restart at zero.

Character-device tests
----------------------

Load and run::

    modprobe rust_chardev
    rs485-test chardev --device /dev/rust-chardev
    modprobe -r rust_chardev

Each open file gets an independent 4096-byte FIFO; dup() and fork() retain
the existing file's FIFO. read/write are stream operations. Empty reads
and full writes wait interruptibly, or return EAGAIN with O_NONBLOCK.
Short copies consume/produce only bytes successfully copied. Zero-length
I/O returns zero, positioned I/O returns ESPIPE, and unsupported ioctls
are handled by VFS. poll/epoll readiness reflects the FIFO predicates.

Holding a file open pins the module. This is why the miscellaneous-device
abstraction now requires an owning module and offers a stream-open option
and poll callback.

UART module removal has different semantics: serial core hangs up active
TTY files and revokes controller access. The hardware test verified HUP,
EOF/EIO on read and EIO on write after removal, followed by successful
module reload. An open UART file need not prevent low-level module removal.

The kselftest entry point is ``make run_tests`` from this test directory.
UART tests are opt-in through ``RS485_DEVICE=/dev/ttyRU0`` after starting
the peer. Missing devices produce kselftest SKIP, not a false PASS.

Validation boundaries
---------------------

Host tests validate the exact shared ring/register-encoding source, CRC,
malformed frames, and PTY fragmentation/deadlines/disconnection. QEMU can
exercise the actual ARM64 character module, user faults, and repeated
module load/unload, including rejection of unload with an open file.
QEMU virt does not emulate the RK3588 UART; physical controller and RS485
tests must be recorded separately on the board. Compile/PTY/QEMU success
must not be presented as proof of board-level UART operation.

Community references
--------------------

* ``Documentation/driver-api/serial/driver.rst`` (serial core / uart_ops).
* ``Documentation/driver-api/misc_devices.rst`` (miscellaneous devices).
* ``Documentation/rust/general-information.rst`` (abstractions vs bindings).
* ``Documentation/rust/coding-guidelines.rst`` (Rust style and safety comments).
* ``Documentation/dev-tools/kselftest.rst`` (test layout and SKIP behavior).
