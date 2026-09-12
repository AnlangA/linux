.. SPDX-License-Identifier: GPL-2.0-only

Rust UART and character-device development on ATK-DLRK3588
=========================================================

Architecture and scope
----------------------

``rust_dw_uart`` is a UART **controller** driver. Its register accesses,
interrupt dispatch, FIFO service, DMA transfer ownership, baud
divisor/framing changes, and break control are implemented in Rust. It
registers a ``ttyRU`` TTY driver with
the existing C serial core for the module's lifetime and adds one line per
bound UART (``ttyRU0`` for the first). It does not call the C 8250
controller's interrupt or register-access implementation.

``rust/kernel/serial.rs`` separates the two lifetimes deliberately. A
``serial::Driver`` lives in a ``static`` and owns the TTY major and the
per-line state that open TTYs keep referring to until they are closed; a
``serial::Port`` is created in ``probe`` and only adds/removes its line.
Because serial core frees line state in ``uart_unregister_driver()``, the
abstraction sets the TTY driver's owner to the module so an open line pins
the module, and it leaks the TTY driver rather than unregistering it while
lines still exist. Controller callbacks receive only the controller state,
never a partially initialized registration object.

``rust_chardev`` is an independent miscellaneous character-device driver.
It provides a bounded FIFO for learning and regression testing; it is not
a wrapper around ``ttyRU0`` and does not access the RS485 hardware.

The serial core and DMAengine framework remain C. Thin C helpers expose
their inline functions and macros to Rust. PL330 also gains a
``device_synchronize`` operation to wait for completion callbacks after
termination; stopping hardware alone does not wait for callbacks already
dispatched by its tasklet. Rust/C pointer and lifetime handling is confined
to the kernel abstractions.

This is a local development implementation, not a claim of upstream
acceptance or production qualification. Current controller support is
RK3588, 32-bit MMIO with a register shift of two, PL330 DMA with interrupt
PIO fallback, and up to ten non-console ports. Console integration, modem
flow control,
automatic runtime power management, and system suspend are outside its
current scope. Do not suspend the system with this driver bound.

The baud clock is enabled and rate-protected while bound. Standard rates
run from the 24 MHz reference; for rates it cannot divide within tolerance
(230400, 460800, 921600, custom rates) the driver asks the CRU for sixteen
times the baud rate before reprogramming the divisor, gating the clock
around the change as ``8250_dw`` does. Fractional-divisor width and FIFO
depth are discovered from the controller. Divisors are rounded with
fractional carry, and baud rates that still have more than two percent
quantization error are rejected by restoring the prior termios and clock;
a rejected initial configuration is logged. The application checks the
effective settings after configuring the TTY.

The receive FIFO triggers at a quarter of its depth and the interrupt
handler drains it completely within a bounded budget; DesignWare's
four-character receive timeout delivers shorter messages. Error bits read
from LSR outside the receive path are preserved for the character they
describe, as the 8250 driver does.

Layout
------

* ``drivers/tty/serial/rust_dw_uart.rs``: UART platform/controller driver.
* ``drivers/tty/serial/rust_dw_uart/config.rs``: register encoding.
* ``rust/kernel/serial.rs``: serial-core registration and locked port API.
* ``rust/kernel/serial/dma.rs``: DMA transfer and cancellation state machine.
* ``rust/kernel/serial/dma/state.rs``: completion bookkeeping shared by tests.
* ``rust/kernel/dmaengine.rs``: private slave-channel and buffer ownership API.
* ``rust/helpers/serial.c``: inline/macro compatibility wrappers.
* ``rust/helpers/dmaengine.c``: DMAengine and workqueue inline helpers.
* ``drivers/dma/pl330.c``: DMA provider, including callback synchronization.
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
``CONFIG_DMA_ENGINE=y``, ``CONFIG_PL330_DMA=y``,
``CONFIG_SERIAL_RUST_DW=m``, ``CONFIG_RUST_CHARDEV=m`` and module unloading.
Enable ``CONFIG_DEBUG_FS=y`` for DMA diagnostic counters.
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
and zero DLF bits. Internal loopback has been verified on the board at
9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 and 1500000
baud; the CRU clock adaptation was observed switching ``sclk_uart3`` to
3686400, 7372800 and 14745600 Hz for the three non-24 MHz rates. RS485
round trips against a CH340 adapter pass at 115200, 230400 and 460800
baud. At 921600 baud the RS485 link fails identically with this driver and
with the C 8250 driver, so that is a limit of the board's transceiver path
or the adapter, not of the controller driver. The driver still rejects
excessive divisor error instead of silently running at a wrong rate.

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
The resulting node is ``/dev/ttyRU0`` (lines are handed out lowest-free
first, so the first bound UART is line 0); ``/dev/ttyS3`` belongs to the
former 8250 driver and must not be used concurrently.

After closing the UART application, restore the default controller::

    tools/testing/selftests/rust_uart_rs485/bind-uart3.sh --restore

DMA operation
-------------

DMA is enabled by default. The driver requests the existing firmware
``tx`` and ``rx`` channels, configures one-byte peripheral accesses, and
allocates coherent bounce buffers in the DMA provider's address domain.
The RX burst matches the UART's quarter-FIFO trigger, capped at 16 bytes.
This implementation accepts PL330 only: its synchronous pause, residue,
non-failing termination and callback synchronization are required by the
ownership model. An unavailable or unsuitable channel selects PIO for the
port; a provider still probing defers the UART probe.

TX copies at most 256 bytes from the TTY queue without advancing it. Only
the completion callback advances the queue and wakes writers. A software
stop allows the issued block to complete, then prevents chaining: PL330
cannot resume a paused transfer. The stop latency therefore depends on
the current baud rate and includes bytes already in the UART FIFO.
``tx_empty`` includes DMA state and UART TEMT so a drain waits for actual
transmission. A transmit flush discards the pending queue advancement
before cancelling DMA; a late callback cannot consume newly queued bytes.

RX uses a 512-byte buffer. Full completions insert the block into the TTY
flip buffer. A UART receive timeout pauses DMA, reads its stable residue,
delivers the received prefix, and drains the remaining FIFO using PIO.
A 20 ms delayed-work timer also flushes short packets whose last DMA
burst emptied the FIFO and therefore generated no UART timeout. This is
a scheduling interval, not a hard latency guarantee. Tiny packets and
FIFO tails can use PIO even with DMA enabled. A line-status error selects
RX PIO for the rest of that open session; DMA has no per-byte error data.

All state transitions hold the UART port lock. Cancelled buffers remain
unavailable until a process-context worker synchronizes the old callback.
Coherent allocation removes explicit cache maintenance, but DMA write/read
barriers still order buffer ownership transfers. Close disables requeueing,
stops interrupts, cancels the worker and synchronizes both channels before
serial core retires its port state. Termios changes prevent new submissions
and quiesce DMA before changing the clock and framing; an interrupted or
overlong TX wait retains the previous settings. Applications should use
``TCSADRAIN`` when bytes already on the wire must retain the old framing.

With debugfs mounted, inspect diagnostic counters as root::

    cat /sys/kernel/debug/rust_dw_uart/dma

``dma_ports`` counts bound ports with both channels. ``tx_dma_bytes`` and
``rx_dma_bytes`` count completed or flushed DMA payload bytes, excluding
PIO tails. ``tx_dma_blocks`` and ``rx_dma_blocks`` count submissions;
``rx_dma_flushes`` counts partial/timeout cancellation paths and
``tx_dma_flushes`` counts explicit transmit cancellations. ``pio_fallbacks``
counts unsuccessful channel setup attempts, while ``dma_errors`` counts
runtime DMA failures and receive line-status errors. Counters are
module-wide and reset on module reload; a concurrent snapshot is not
atomic across fields. This debugfs text is diagnostic, not a stable ABI.

To compare PIO, close applications, stop any automatic binding service,
restore the C controller with the binding helper, then reload::

    modprobe -r rust_dw_uart
    modprobe rust_dw_uart dma=0
    tools/testing/selftests/rust_uart_rs485/bind-uart3.sh --rust

Repeat the restore/unload/load/bind sequence without ``dma=0`` to return
to DMA. Loading the module again while it is already loaded does not
change this parameter. Restart the binding service if one is installed.

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

UART device *unbinding* has different semantics: serial core hangs up
active TTY files and revokes controller access, so readers see HUP and
EOF/EIO and writers see EIO, while the line state stays owned by the
module-lifetime TTY driver until the file is closed. UART *module removal*
behaves like the character device: an open ``ttyRU`` file holds a module
reference and ``rmmod`` fails with EBUSY until it is closed. Earlier builds
allowed removal with an open file, which freed line state that the TTY
still referenced; that was a use-after-free, not a feature.

The kselftest entry point is ``make run_tests`` from this test directory.
UART tests are opt-in through ``RS485_DEVICE=/dev/ttyRU0`` after starting
the peer. Missing devices produce kselftest SKIP, not a false PASS.

Validation boundaries
---------------------

Host tests validate the exact shared ring/register-encoding source, DMA
completion/cancellation bookkeeping, CRC,
malformed frames, and PTY fragmentation/deadlines/disconnection. QEMU can
exercise the actual ARM64 character module, user faults, and repeated
module load/unload, including rejection of unload with an open file.
QEMU virt does not emulate the RK3588 UART; physical controller and RS485
tests must be recorded separately on the board. DMA validation additionally
requires nonzero TX/RX counters, full RX completions and short-message
timeouts, cancellation while active, device unbind/rebind, live termios
changes, software stop/start and explicit PIO comparison. QEMU virt does
not test the RK3588 PL330 channels. Compile/PTY/QEMU success
must not be presented as proof of board-level UART operation.

Community references
--------------------

* ``Documentation/driver-api/serial/driver.rst`` (serial core / uart_ops).
* ``Documentation/driver-api/dmaengine/client.rst`` (DMAengine client lifecycle).
* ``Documentation/core-api/dma-api-howto.rst`` (DMA address domains and ordering).
* ``Documentation/driver-api/misc_devices.rst`` (miscellaneous devices).
* ``Documentation/rust/general-information.rst`` (abstractions vs bindings).
* ``Documentation/rust/coding-guidelines.rst`` (Rust style and safety comments).
* ``Documentation/dev-tools/kselftest.rst`` (test layout and SKIP behavior).
