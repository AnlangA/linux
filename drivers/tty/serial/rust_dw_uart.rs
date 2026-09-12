// SPDX-License-Identifier: GPL-2.0-only

//! Interrupt-driven Rust controller for RK3588's DesignWare APB UART.
//!
//! Bind explicitly using driver_override. There is intentionally no automatic
//! OF match table: loading this development driver must not capture a console.

use kernel::{
    clk::{
        Clk,
        ExclusiveEnabledClk,
        Hertz, //
    },
    device::Core,
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
};

#[path = "rust_dw_uart/config.rs"]
mod config;

const FIFO_SIZE: u32 = 64;
const IRQ_BUDGET: usize = 256;
const IER_RX: u32 = 1;
const IER_TX: u32 = 2;
const IER_LSR: u32 = 4;

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
    io: IoMem<'a, 0x100>,
    dlf_bits: u8,
    // The APB clock outlives baud-clock teardown; both outlive all callbacks.
    _baud_clock: ExclusiveEnabledClk,
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
    fn lsr(&self) -> u32 {
        self.io.read32(0x14)
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
            self.io.write32(0x07, 0x08); // Enable and clear both FIFOs.
            let _ = self.io.read32(0x00);
        }
        Err(EBUSY)
    }

    fn transmit(&self, port: &mut LockedPort<'_>) {
        // TFNF is stronger than THRE: it also permits filling a partially full FIFO.
        for _ in 0..FIFO_SIZE {
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
}

impl Hardware for DwUart<'_> {
    fn tx_empty(&self) -> bool {
        self.lsr() & 0x40 != 0
    }

    fn set_loopback(&self, enable: bool) {
        // No modem lines are routed through JP6. Only honor TIOCM_LOOP.
        self.io.write32(0x08 | if enable { 0x10 } else { 0 }, 0x10);
    }

    fn startup(&self) -> Result {
        self.set_ier(0);
        self.io.write32(0x07, 0x08);
        let _ = self.lsr();
        let _ = self.io.read32(0x00);
        let _ = self.io.read32(0x08);
        let _ = self.io.read32(0x18);
        self.set_ier(IER_RX | IER_LSR);
        Ok(())
    }

    fn shutdown(&self) {
        self.set_ier(0);
        let _ = self.write_lcr(self.lcr() & !0x40);
        self.io.write32(0x07, 0x08);
    }

    fn start_tx(&self, port: &mut LockedPort<'_>) {
        self.set_ier(self.ier() | IER_TX);
        self.transmit(port);
    }

    fn stop_tx(&self) {
        self.set_ier(self.ier() & !IER_TX);
    }
    fn flush_tx(&self) {
        self.stop_tx();
        self.io.write32(0x05, 0x08); // Keep RX FIFO; clear the TX FIFO only.
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
        self.io.write32(0x01, 0x08); // FIFO enabled, lowest RX trigger for latency.
        self.set_ier(ier);
        Ok(divisor.actual_baud)
    }

    fn interrupt(&self, port: &mut LockedPort<'_>) -> bool {
        let mut handled = false;
        let mut received = false;
        for _ in 0..IRQ_BUDGET {
            let id = self.io.read32(0x08) & 0x0f;
            if id == 1 {
                break;
            }
            handled = true;
            match id {
                2 => self.transmit(port),
                4 | 6 | 12 => {
                    // Drain one byte per iteration so an RX flood cannot create
                    // an unbounded hard-IRQ loop. Level-triggered IRQ retriggers.
                    let lsr = self.lsr();
                    if lsr & 0x11 != 0 {
                        let byte = if lsr & 1 != 0 {
                            self.io.read32(0x00) as u8
                        } else {
                            0
                        };
                        port.receive(byte, config::receive_status(lsr));
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
    type Data<'bound> = serial::Registration<'bound, DwUart<'bound>>;

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
            let baud = ExclusiveEnabledClk::new(baud, Hertz::from_mhz(24))?;
            let rate = u32::try_from(baud.rate().as_hz()).map_err(|_| EINVAL)?;
            if rate < 16 * 115200 {
                return Err(EINVAL);
            }
            // Ensure the old driver did not leave DLAB selected before masking IRQs.
            let mut hardware = DwUart {
                io,
                dlf_bits: 0,
                _baud_clock: baud,
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
            let fifo_size = if depth == 0 { FIFO_SIZE } else { depth };
            dev_info!(
                dev,
                "Rust UART controller: ttyRU0, clock {} Hz, FIFO {}, DLF bits {}\n",
                rate,
                fifo_size,
                bits
            );
            Ok(serial::Registration::new(
                dev,
                pdev.irq_by_index(0)?,
                hardware,
                mapbase,
                rate,
                fifo_size,
                &THIS_MODULE,
            ))
        })
    }
}

kernel::module_platform_driver! {
    type: RustDwUart,
    name: "rust_dw_uart",
    authors: ["ATK Rust driver contributors"],
    description: "Rust RK3588 DesignWare UART controller (explicit binding)",
    license: "GPL v2",
}
