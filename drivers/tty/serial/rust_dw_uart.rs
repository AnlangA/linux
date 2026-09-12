// SPDX-License-Identifier: GPL-2.0-only

//! Interrupt-driven Rust controller for RK3588's DesignWare APB UART.
//!
//! Bind explicitly using driver_override. There is intentionally no automatic
//! OF match table: loading this development driver must not capture a console.
//!
//! The TTY driver (`/dev/ttyRU*`) lives for the module's lifetime; each bound
//! UART adds one line. Unbinding a device hangs its line up, while an open
//! line keeps the module loaded.

use kernel::{
    clk::{
        Clk,
        ExclusiveEnabledClk,
        Hertz, //
    },
    debugfs,
    device::Core,
    driver,
    io::{
        mem::IoMem,
        Io, //
    },
    platform,
    prelude::*,
    serial::{
        self,
        Hardware,
        LineConfig,
        LockedPort, //
    },
    sync::atomic::{
        Atomic,
        Relaxed, //
    },
};

#[path = "rust_dw_uart/config.rs"]
mod config;

/// RK3588 has UART0 to UART9.
const LINES: u32 = 10;
/// The fixed 24 MHz reference every standard rate up to 1.5 Mbaud divides from.
const REFERENCE_RATE: u32 = 24_000_000;
const FIFO_SIZE: u32 = 64;
const IRQ_BUDGET: usize = 256;
const IER_RX: u32 = 1;
const IER_TX: u32 = 2;
const IER_LSR: u32 = 4;
/// FIFOs enabled with the receive trigger at a quarter depth; DesignWare's
/// four-character timeout interrupt still delivers shorter messages promptly.
const FCR_ENABLE: u32 = 0x49;
const FCR_RESET_ALL: u32 = FCR_ENABLE | 0x06;
const FCR_RESET_TX: u32 = FCR_ENABLE | 0x04;
/// LSR overrun, parity, framing and break bits.
const LSR_ERRORS: u32 = 0x1e;
/// LSR data-ready and break bits.
const LSR_DATA: u32 = 0x11;

static TTY_DRIVER: serial::Driver = serial::Driver::new(serial::DriverInfo {
    driver_name: c"rust_dw_uart",
    dev_name: c"ttyRU",
    lines: LINES,
});

/// Widens a register-sized rate; the driver is ARM64-only, so this never saturates.
fn hertz(rate: u32) -> Hertz {
    Hertz(c_ulong::try_from(rate).unwrap_or(c_ulong::MAX))
}

/// A clock kept enabled for the entire period that registers can be accessed.
struct EnabledClock(Clk);

impl EnabledClock {
    fn new(clk: Clk) -> Result<Self> {
        clk.prepare_enable()?;
        Ok(Self(clk))
    }
}

impl Drop for EnabledClock {
    fn drop(&mut self) {
        self.0.disable_unprepare();
    }
}

/// All register accesses are serialized by serial core's UART port spinlock.
struct DwUart<'a> {
    dma: Option<serial::dma::Config>,
    io: IoMem<'a, 0x100>,
    dlf_bits: u8,
    fifo_size: u32,
    /// LSR error bits describe the FIFO head and clear on read. Reads outside
    /// the receive path park them here so no error is lost (8250's
    /// `lsr_saved_flags`). Only touched under the port lock.
    saved_lsr: Atomic<u32>,
    // The APB clock outlives baud-clock teardown; both outlive all callbacks.
    baud_clock: ExclusiveEnabledClk,
    _bus_clock: EnabledClock,
}

impl DwUart<'_> {
    fn ier(&self) -> u32 {
        self.io.read32(0x04)
    }
    fn set_ier(&self, value: u32) {
        self.io.write32(value, 0x04);
    }
    fn lcr(&self) -> u32 {
        self.io.read32(0x0c)
    }

    /// Reads LSR outside the receive path, keeping error bits for it.
    fn lsr(&self) -> u32 {
        let lsr = self.io.read32(0x14);
        self.saved_lsr
            .store(self.saved_lsr.load(Relaxed) | (lsr & LSR_ERRORS), Relaxed);
        lsr
    }

    /// DesignWare may reject LCR writes while busy. Match the established
    /// driver's bounded retry/force-idle sequence, including FIFO resets.
    fn write_lcr(&self, value: u32) -> Result {
        for _ in 0..1000 {
            self.io.write32(value, 0x0c);
            if self.lcr() == value {
                return Ok(());
            }
            let _ = self.io.read32(0x7c); // Clear busy-detect interrupt.
            self.io.write32(FCR_RESET_ALL, 0x08);
            let _ = self.io.read32(0x00);
        }
        Err(EBUSY)
    }

    fn transmit(&self, port: &mut LockedPort<'_>) {
        // TFNF is stronger than THRE: it also permits filling a partially full FIFO.
        for _ in 0..self.fifo_size {
            if self.io.read32(0x7c) & 0x02 == 0 {
                break;
            }
            let Some(byte) = port.next_tx() else {
                self.stop_tx();
                break;
            };
            self.io.write32(u32::from(byte), 0x00);
        }
        port.wake_writers();
    }

    /// Drains the receive FIFO within `budget` and reports whether it read anything.
    fn receive(&self, port: &mut LockedPort<'_>, budget: &mut usize) -> bool {
        let mut lsr = self.io.read32(0x14) | self.saved_lsr.xchg(0, Relaxed);
        let mut received = false;
        while lsr & LSR_DATA != 0 && *budget > 0 {
            *budget -= 1;
            let byte = if lsr & 0x01 != 0 {
                self.io.read32(0x00) as u8
            } else {
                0
            };
            port.receive(byte, config::receive_status(lsr));
            received = true;
            lsr = self.io.read32(0x14);
        }
        received
    }
}

impl Hardware for DwUart<'_> {
    fn dma_config(&self) -> Option<serial::dma::Config> {
        self.dma
    }

    fn drain_rx(&self, port: &mut LockedPort<'_>) {
        let mut budget = IRQ_BUDGET;
        if self.receive(port, &mut budget) {
            port.push_rx();
        }
    }

    fn tx_empty(&self) -> bool {
        self.lsr() & 0x40 != 0
    }

    fn set_loopback(&self, enable: bool) {
        // No modem lines are routed through JP6. Only honor TIOCM_LOOP.
        self.io.write32(0x08 | if enable { 0x10 } else { 0 }, 0x10);
    }

    fn startup(&self) -> Result {
        self.set_ier(0);
        self.io.write32(FCR_RESET_ALL, 0x08);
        let _ = self.io.read32(0x14);
        self.saved_lsr.store(0, Relaxed);
        let _ = self.io.read32(0x00);
        let _ = self.io.read32(0x08);
        let _ = self.io.read32(0x18);
        self.set_ier(IER_RX | IER_LSR);
        Ok(())
    }

    fn shutdown(&self) {
        self.set_ier(0);
        let _ = self.write_lcr(self.lcr() & !0x40);
        self.io.write32(FCR_RESET_ALL, 0x08);
    }

    fn start_tx(&self, port: &mut LockedPort<'_>) {
        if port.dma_tx() {
            self.stop_tx();
            return;
        }
        self.set_ier(self.ier() | IER_TX);
        self.transmit(port);
    }

    fn stop_tx(&self) {
        self.set_ier(self.ier() & !IER_TX);
    }
    fn flush_tx(&self) {
        self.stop_tx();
        self.io.write32(FCR_RESET_TX, 0x08); // Keep RX FIFO; clear the TX FIFO only.
    }
    fn stop_rx(&self) {
        self.set_ier(self.ier() & !(IER_RX | IER_LSR));
    }

    fn set_break(&self, enable: bool) {
        let value = if enable {
            self.lcr() | 0x40
        } else {
            self.lcr() & !0x40
        };
        let _ = self.write_lcr(value);
    }

    fn prepare_clock(&self, baud: u32, clock: u32) -> u32 {
        // Standard rates keep the 24 MHz reference. Others are synthesized by
        // the CRU at sixteen times the baud rate, as 8250_dw does; if that
        // fails the divisor check in configure() rejects the rate cleanly.
        let wanted = hertz(config::baud_clock(REFERENCE_RATE, baud, self.dlf_bits));
        if self.baud_clock.rate() != wanted {
            let _ = self.baud_clock.set_rate(wanted);
        }
        u32::try_from(self.baud_clock.rate().as_hz()).unwrap_or(clock)
    }

    fn configure(&self, line: LineConfig) -> Result<u32> {
        let divisor = config::divisor(line.clock, line.baud, self.dlf_bits).ok_or(EINVAL)?;
        let old_lcr = self.lcr() & !0x80;
        let ier = self.ier();
        self.set_ier(0);
        let lcr = config::line_control(
            line.data_bits,
            line.two_stop_bits,
            line.parity,
            line.odd_parity,
        );
        if let Err(err) = self.write_lcr(lcr | 0x80) {
            let _ = self.write_lcr(old_lcr);
            self.set_ier(ier);
            return Err(err);
        }
        let old_low = self.io.read32(0x00);
        let old_high = self.io.read32(0x04);
        let old_fraction = self.io.read32(0xc0);
        self.io.write32(u32::from(divisor.integer) & 0xff, 0x00);
        self.io.write32(u32::from(divisor.integer) >> 8, 0x04);
        self.io.write32(divisor.fraction, 0xc0);
        if let Err(err) = self.write_lcr(lcr) {
            // Restore divisor and framing before reporting failure to serial core.
            if self.write_lcr(old_lcr | 0x80).is_ok() {
                self.io.write32(old_low, 0x00);
                self.io.write32(old_high, 0x04);
                self.io.write32(old_fraction, 0xc0);
            }
            let _ = self.write_lcr(old_lcr);
            self.set_ier(ier);
            return Err(err);
        }
        self.io.write32(FCR_ENABLE, 0x08);
        self.set_ier(ier);
        Ok(divisor.actual_baud)
    }

    fn interrupt(&self, port: &mut LockedPort<'_>) -> bool {
        let mut handled = false;
        let mut received = false;
        // One budget bounds IIR iterations and received bytes together, so an
        // RX flood cannot create an unbounded hard-IRQ loop. The level-
        // triggered IRQ simply fires again for whatever is left.
        let mut budget = IRQ_BUDGET;
        while budget > 0 {
            budget -= 1;
            let id = self.io.read32(0x08) & 0x0f;
            if id == 1 {
                break;
            }
            handled = true;
            match id {
                2 => self.start_tx(port),
                4 if port.dma_rx() => break,
                4 | 6 | 12 => {
                    if id == 6 && !port.dma_rx_error() {
                        break;
                    }
                    if id == 12 && !port.dma_rx_flush() {
                        break;
                    }
                    if self.receive(port, &mut budget) {
                        received = true;
                    } else if id == 12 {
                        // DW spurious receive-timeout quirk: dummy RBR clears it.
                        let _ = self.io.read32(0x00);
                    }
                }
                7 => {
                    let _ = self.io.read32(0x7c);
                }
                0 => {
                    let _ = self.io.read32(0x18);
                }
                _ => break,
            }
        }
        if received {
            port.push_rx();
        }
        handled
    }
}

struct RustDwUart;

impl platform::Driver for RustDwUart {
    type IdInfo = ();
    type Data<'bound> = serial::Port<'bound, DwUart<'bound>>;

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        pin_init::pin_init_scope(move || {
            let dev = pdev.as_ref();
            let node = dev.fwnode().ok_or(ENODEV)?;
            node.property_match_string(c"compatible", c"rockchip,rk3588-uart")?;
            let resource = pdev.resource_by_index(0).ok_or(ENODEV)?;
            let mapbase = resource.start();
            let io = pdev
                .io_request_by_index(0)
                .ok_or(ENODEV)?
                .iomap_sized::<0x100>()?;
            let bus = EnabledClock::new(Clk::get(dev, Some(c"apb_pclk"))?)?;
            let baud = Clk::get(dev, Some(c"baudclk"))?;
            let baud = ExclusiveEnabledClk::new(baud, hertz(REFERENCE_RATE))?;
            let rate = u32::try_from(baud.rate().as_hz()).map_err(|_| EINVAL)?;
            if rate < 16 * 115200 {
                return Err(EINVAL);
            }
            // Ensure the old driver did not leave DLAB selected before masking IRQs.
            let mut hardware = DwUart {
                dma: None,
                io,
                dlf_bits: 0,
                fifo_size: FIFO_SIZE,
                saved_lsr: Atomic::new(0),
                baud_clock: baud,
                _bus_clock: bus,
            };
            hardware.write_lcr(3)?;
            hardware.set_ier(0);
            // Discover DLF width using the same capability test as 8250_dwlib.
            // The port is unbound, interrupts are masked, and the value is restored.
            let old_dlf = hardware.io.read32(0xc0);
            hardware.io.write32(u32::MAX, 0xc0);
            let mask = hardware.io.read32(0xc0);
            hardware.io.write32(old_dlf, 0xc0);
            let bits = mask.count_ones() as u8;
            if bits > 16 || u64::from(mask) != (1u64 << bits) - 1 {
                return Err(ENODEV);
            }
            hardware.dlf_bits = bits;
            let depth = ((hardware.io.read32(0xf4) >> 16) & 0xff) * 16;
            if depth != 0 {
                hardware.fifo_size = depth;
            }
            let fifo_size = hardware.fifo_size;
            if *module_parameters::dma.value() != 0 {
                hardware.dma = Some(serial::dma::Config {
                    fifo: mapbase,
                    burst: (fifo_size / 4).clamp(1, 16),
                });
            }
            Ok(serial::Port::new(
                &TTY_DRIVER,
                dev,
                pdev.irq_by_index(0)?,
                hardware,
                mapbase,
                rate,
                fifo_size,
            )
            .pin_chain(move |port| {
                dev_info!(
                    dev,
                    "Rust UART controller: {}{}, clock {} Hz, FIFO {}, DLF bits {}, DMA {}\n",
                    TTY_DRIVER.info().dev_name,
                    port.line(),
                    rate,
                    fifo_size,
                    bits,
                    port.dma_enabled()
                );
                Ok(())
            }))
        })
    }
}

/// The TTY driver is registered first and torn down last, so that every
/// device is unbound (and its line removed) while the driver still exists.
#[pin_data]
struct RustDwUartModule {
    #[pin]
    dma_stats: debugfs::File<&'static serial::dma::Stats>,
    #[pin]
    platform: driver::Registration<platform::Adapter<RustDwUart>>,
    tty: serial::Registration,
    debugfs: debugfs::Dir,
}

impl kernel::InPlaceModule for RustDwUartModule {
    fn init(module: &'static ThisModule) -> impl PinInit<Self, Error> {
        let debugfs = debugfs::Dir::new(c"rust_dw_uart");
        try_pin_init!(Self {
            tty: TTY_DRIVER.register(module)?,
            dma_stats <- debugfs.read_callback_file(c"dma", TTY_DRIVER.dma_stats(),
                &|stats, f| stats.write(f)),
            platform <- driver::Registration::new(
                <Self as kernel::ModuleMetadata>::NAME,
                module,
            ),
            debugfs,
        })
    }
}

module! {
    type: RustDwUartModule,
    name: "rust_dw_uart",
    authors: ["ATK Rust driver contributors"],
    description: "Rust RK3588 DesignWare UART controller (explicit binding)",
    license: "GPL v2",
    params: {
        dma: u32 {
            default: 1,
            description: "Use firmware DMA channels when available (0 selects PIO)",
        },
    },
}
