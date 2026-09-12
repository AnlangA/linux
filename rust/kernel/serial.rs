// SPDX-License-Identifier: GPL-2.0-only

//! Minimal interrupt-driven UART integration with serial core.
//!
//! A registration owns one TTY line and revokes all callbacks before its
//! hardware resources are dropped. The abstraction deliberately exposes no
//! raw uart_port pointer to controller drivers.

use crate::{
    bindings,
    device::{
        Bound,
        Device, //
    },
    error::to_result,
    irq::IrqRequest,
    prelude::*,
    types::Opaque,
    ThisModule, //
};
use core::{
    marker::PhantomData,
    ptr, //
};

/// Error flags accepted by [`LockedPort::receive`].
pub mod receive {
    /// Hardware receive overrun.
    pub const OVERRUN: u32 = 1;
    /// Incorrect parity.
    pub const PARITY: u32 = 2;
    /// Invalid stop bit.
    pub const FRAME: u32 = 4;
    /// Break condition.
    pub const BREAK: u32 = 8;
    pub(super) const DATA: u32 = 16;
}

/// UART framing requested through termios.
#[derive(Clone, Copy)]
pub struct LineConfig {
    /// Requested baud rate selected by serial core.
    pub baud: u32,
    /// Stable input clock rate in Hz.
    pub clock: u32,
    /// Five, six, seven, or eight data bits.
    pub data_bits: u8,
    /// Request two stop bits (1.5 for five-bit UART framing).
    pub two_stop_bits: bool,
    /// Parity enabled.
    pub parity: bool,
    /// Odd parity when parity is enabled.
    pub odd_parity: bool,
}

/// Controller operations, always called with the UART spinlock held.
///
/// Methods must not sleep. IRQ service must have a bounded work budget.
pub trait Hardware: Send + Sync {
    /// True only after both FIFO and shift register have drained.
    fn tx_empty(&self) -> bool;
    /// Sets outputs; only internal loopback is exposed by this initial API.
    fn set_loopback(&self, enable: bool);
    /// Initializes the UART and enables receive interrupts.
    fn startup(&self) -> Result;
    /// Disables all interrupt sources and cancels break.
    fn shutdown(&self);
    /// Begins/continues draining serial core's transmit queue.
    fn start_tx(&self, port: &mut LockedPort<'_>);
    /// Disables the transmit-empty interrupt.
    fn stop_tx(&self);
    /// Discards queued hardware transmit bytes when TTY requests an output flush.
    fn flush_tx(&self);
    /// Stops reception during close.
    fn stop_rx(&self);
    /// Sets or clears break.
    fn set_break(&self, enable: bool);
    /// Configures framing/divisor, or leaves the previous configuration intact.
    fn configure(&self, config: LineConfig) -> Result<u32>;
    /// Services a UART interrupt and reports whether this UART caused it.
    fn interrupt(&self, port: &mut LockedPort<'_>) -> bool;
}

/// A serial-core port borrowed while its spinlock is held and state is live.
pub struct LockedPort<'a> {
    raw: *mut bindings::uart_port,
    _lifetime: PhantomData<&'a mut bindings::uart_port>,
}

impl LockedPort<'_> {
    /// Pops an XON/XOFF byte first, then an ordinary queued byte.
    pub fn next_tx(&mut self) -> Option<u8> {
        // SAFETY: The callback contract guarantees a locked port and live state.
        unsafe {
            if (*self.raw).x_char != 0 {
                let byte = (*self.raw).x_char;
                (*self.raw).x_char = 0;
                (*self.raw).icount.tx = (*self.raw).icount.tx.wrapping_add(1);
                return Some(byte);
            }
            if bindings::uart_tx_stopped(self.raw) != 0 {
                return None;
            }
            let mut byte = 0;
            (bindings::uart_fifo_get(self.raw, &mut byte) != 0).then_some(byte)
        }
    }

    /// Wakes blocked writers when the serial-core queue is running low.
    pub fn wake_writers(&mut self) {
        // SAFETY: Access to the transmit queue is serialized by the port lock.
        unsafe {
            if bindings::uart_xmit_pending(self.raw) < bindings::WAKEUP_CHARS {
                bindings::uart_write_wakeup(self.raw);
            }
        }
    }

    /// Accounts a received byte/error and inserts it using termios filtering.
    pub fn receive(&mut self, byte: u8, mut status: u32) {
        // SAFETY: The caller holds the port lock and this port's state is live.
        unsafe {
            let count = &mut (*self.raw).icount;
            count.rx = count.rx.wrapping_add(1);
            if status & receive::BREAK != 0 {
                count.brk = count.brk.wrapping_add(1);
                status &= !(receive::PARITY | receive::FRAME);
                if bindings::uart_handle_break(self.raw) != 0 {
                    return;
                }
            } else if status & receive::PARITY != 0 {
                count.parity = count.parity.wrapping_add(1);
            } else if status & receive::FRAME != 0 {
                count.frame = count.frame.wrapping_add(1);
            }
            if status & receive::OVERRUN != 0 {
                (*self.raw).icount.overrun = (*self.raw).icount.overrun.wrapping_add(1);
            }
            status = (status | receive::DATA) & (*self.raw).read_status_mask;
            let flag = if status & receive::BREAK != 0 {
                bindings::TTY_BREAK
            } else if status & receive::PARITY != 0 {
                bindings::TTY_PARITY
            } else if status & receive::FRAME != 0 {
                bindings::TTY_FRAME
            } else {
                bindings::TTY_NORMAL
            };
            bindings::uart_insert_char(self.raw, status, receive::OVERRUN, byte, flag as u8);
        }
    }

    /// Makes accumulated input visible to the TTY line discipline.
    pub fn push_rx(&mut self) {
        // SAFETY: The port state remains live throughout the callback.
        unsafe { bindings::tty_flip_buffer_push(ptr::addr_of_mut!((*(*self.raw).state).port)) };
    }
}

/// One TTY line with dynamically allocated device numbers.
///
/// # Invariants
///
/// Both the UART driver and port are registered and pinned. `hardware` and
/// the parent device outlive all callbacks; removal stops IRQs synchronously.
#[pin_data(PinnedDrop)]
pub struct Registration<'a, T: Hardware> {
    hardware: T,
    #[pin]
    driver: Opaque<bindings::uart_driver>,
    #[pin]
    port: Opaque<bindings::uart_port>,
    _device: PhantomData<&'a Device<Bound>>,
    registered: bool,
}

// SAFETY: Hardware is Send + Sync. All mutable uart_port accesses are protected
// by the serial-core port lock or serialized startup/shutdown/core lifecycle.
unsafe impl<T: Hardware> Send for Registration<'_, T> {}
// SAFETY: The same UART lock serializes concurrent callbacks.
unsafe impl<T: Hardware> Sync for Registration<'_, T> {}

impl<'a, T: Hardware + 'a> Registration<'a, T> {
    /// Registers a single-line controller as `/dev/ttyRU0`.
    ///
    /// The caller must supply MMIO obtained from this device, with interrupt
    /// sources disabled. Only one registration may own this TTY driver name.
    pub fn new(
        dev: &'a Device<Bound>,
        irq: IrqRequest<'a>,
        hardware: T,
        mapbase: u64,
        clock: u32,
        fifo_size: u32,
        module: &'static ThisModule,
    ) -> impl PinInit<Self, Error> + 'a {
        try_pin_init!(&this in Self {
            hardware: hardware,
            driver <- Opaque::try_ffi_init(|slot: *mut bindings::uart_driver| {
                // SAFETY: The initializer exclusively owns this uninitialized slot.
                unsafe {
                    slot.write(pin_init::zeroed());
                    (*slot).owner = module.as_ptr();
                    (*slot).driver_name = c"rust_dw_uart".as_char_ptr();
                    (*slot).dev_name = c"ttyRU".as_char_ptr();
                    (*slot).nr = 1;
                }
                Ok::<(), Error>(())
            }),
            port <- Opaque::try_ffi_init(|slot: *mut bindings::uart_port| {
                // SAFETY: The initializer owns the slot; all stored addresses
                // stay valid until uart_remove_one_port() completes in Drop.
                unsafe {
                    slot.write(pin_init::zeroed());
                    bindings::__spin_lock_init(
                        ptr::addr_of_mut!((*slot).lock),
                        c"rust-uart-port".as_char_ptr(),
                        crate::static_lock_class!().as_ptr(),
                    );
                    (*slot).dev = dev.as_raw();
                    (*slot).irq = irq.irq();
                    (*slot).mapbase = mapbase;
                    (*slot).uartclk = clock;
                    (*slot).fifosize = fifo_size;
                    (*slot).iotype = bindings::uart_iotype_UPIO_MEM32;
                    (*slot).regshift = 2;
                    (*slot).type_ = bindings::PORT_16550A;
                    (*slot).flags = bindings::UPF_FIXED_PORT | bindings::UPF_FIXED_TYPE;
                    (*slot).ops = &Self::OPS;
                    (*slot).private_data = this.as_ptr().cast();
                }
                Ok::<(), Error>(())
            }),
            _device: PhantomData,
            registered: {
                // SAFETY: The two opaque fields and hardware were initialized
                // above. No callback reads the final `registered` field.
                let (driver, port) = unsafe {
                    ((*this.as_ptr()).driver.get(), (*this.as_ptr()).port.get())
                };
                // SAFETY: The initialized UART driver remains pinned until Drop.
                to_result(unsafe { bindings::uart_register_driver(driver) })?;
                // SAFETY: All callbacks and hardware are initialized and live.
                let result = unsafe { bindings::uart_add_one_port(driver, port) };
                if result < 0 {
                    // SAFETY: Balances the successful registration above.
                    unsafe { bindings::uart_unregister_driver(driver) };
                    return Err(Error::from_errno(result));
                }
                true
            },
        })
    }

    /// # Safety
    ///
    /// `port` must belong to this exact registration type, and the registration
    /// must remain alive for the entire returned borrow `'b`.
    unsafe fn from_port<'b>(port: *mut bindings::uart_port) -> &'b Self {
        // SAFETY: Called only with a registered port owned by this instantiation.
        unsafe { &*(*port).private_data.cast::<Self>() }
    }

    fn locked<R>(port: *mut bindings::uart_port, f: impl FnOnce(&mut LockedPort<'_>) -> R) -> R {
        let mut flags = 0;
        // SAFETY: All callers pass a live port owned by this registration.
        unsafe { bindings::uart_port_lock_irqsave(port, &mut flags) };
        let result = f(&mut LockedPort {
            raw: port,
            _lifetime: PhantomData,
        });
        // SAFETY: Balances the lock above on the same CPU and restores its flags.
        unsafe { bindings::uart_port_unlock_irqrestore(port, flags) };
        result
    }

    /// # Safety
    ///
    /// `port` must belong to a live registration; its lock must not be held.
    unsafe extern "C" fn tx_empty(port: *mut bindings::uart_port) -> u32 {
        // SAFETY: serial core only invokes callbacks while the port is registered.
        let this = unsafe { Self::from_port(port) };
        Self::locked(port, |_| {
            if this.hardware.tx_empty() {
                bindings::TIOCSER_TEMT
            } else {
                0
            }
        })
    }

    /// # Safety
    ///
    /// `port` must belong to a live registration and its IRQ-safe lock is held.
    unsafe extern "C" fn set_mctrl(port: *mut bindings::uart_port, ctrl: u32) {
        // SAFETY: serial core calls this with the port lock held.
        unsafe { Self::from_port(port) }
            .hardware
            .set_loopback(ctrl & bindings::TIOCM_LOOP != 0);
    }

    extern "C" fn get_mctrl(_port: *mut bindings::uart_port) -> u32 {
        // The board has no modem input pins on its RS485 connection.
        bindings::TIOCM_CAR | bindings::TIOCM_CTS | bindings::TIOCM_DSR
    }

    /// # Safety
    ///
    /// `port` and its transmit state are live and its IRQ-safe lock is held.
    unsafe extern "C" fn start_tx(port: *mut bindings::uart_port) {
        // SAFETY: serial core supplies its lock and live state for start_tx.
        unsafe { Self::from_port(port) }
            .hardware
            .start_tx(&mut LockedPort {
                raw: port,
                _lifetime: PhantomData,
            });
    }

    /// # Safety
    ///
    /// `port` belongs to a live registration and its IRQ-safe lock is held.
    unsafe extern "C" fn stop_tx(port: *mut bindings::uart_port) {
        // SAFETY: serial core supplies the port lock.
        unsafe { Self::from_port(port) }.hardware.stop_tx();
    }

    /// # Safety
    ///
    /// `port` belongs to a live registration and its IRQ-safe lock is held.
    unsafe extern "C" fn flush_buffer(port: *mut bindings::uart_port) {
        // SAFETY: Serial core invokes this with the live port locked.
        unsafe { Self::from_port(port) }.hardware.flush_tx();
    }

    /// # Safety
    ///
    /// `port` belongs to a live registration and its IRQ-safe lock is held.
    unsafe extern "C" fn stop_rx(port: *mut bindings::uart_port) {
        // SAFETY: serial core supplies the port lock.
        unsafe { Self::from_port(port) }.hardware.stop_rx();
    }

    /// # Safety
    ///
    /// `port` is live, its TTY mutex is held, and its spinlock is not held.
    unsafe extern "C" fn break_ctl(port: *mut bindings::uart_port, enabled: i32) {
        // SAFETY: The registered port stays alive during this callback.
        let this = unsafe { Self::from_port(port) };
        Self::locked(port, |_| this.hardware.set_break(enabled != 0));
    }

    /// # Safety
    ///
    /// `port` and its state are live. Serial core serializes this call against
    /// shutdown and calls it only while no IRQ is registered for this port.
    unsafe extern "C" fn startup(port: *mut bindings::uart_port) -> i32 {
        // SAFETY: serial core serializes startup and shutdown; state is live.
        let this = unsafe { Self::from_port(port) };
        // SAFETY: The port and callback cookie remain live until shutdown's
        // free_irq() synchronizes the handler. Hardware IRQ sources are masked.
        let result = unsafe {
            bindings::request_irq(
                (*port).irq,
                Some(Self::interrupt),
                0,
                c"rust_dw_uart".as_char_ptr(),
                port.cast(),
            )
        };
        if result < 0 {
            return result;
        }
        if let Err(err) = Self::locked(port, |_| this.hardware.startup()) {
            // SAFETY: Balances request_irq; startup has not been published.
            unsafe { bindings::free_irq((*port).irq, port.cast()) };
            return err.to_errno();
        }
        0
    }

    /// # Safety
    ///
    /// Exactly one successful startup must precede this call. State and
    /// hardware must remain alive until this synchronous shutdown completes.
    unsafe extern "C" fn shutdown(port: *mut bindings::uart_port) {
        // SAFETY: serial core invokes shutdown once for each successful startup.
        let this = unsafe { Self::from_port(port) };
        Self::locked(port, |_| this.hardware.shutdown());
        // SAFETY: Hardware sources are now masked. free_irq waits for an in-flight
        // handler before serial core is allowed to destroy the port state.
        unsafe { bindings::free_irq((*port).irq, port.cast()) };
    }

    /// # Safety
    ///
    /// `cookie` must be the live port supplied to request_irq by startup.
    unsafe extern "C" fn interrupt(_irq: i32, cookie: *mut c_void) -> bindings::irqreturn_t {
        let port = cookie.cast::<bindings::uart_port>();
        // SAFETY: request_irq/free_irq bound this cookie's lifetime to live state.
        let this = unsafe { Self::from_port(port) };
        if Self::locked(port, |locked| this.hardware.interrupt(locked)) {
            bindings::irqreturn_IRQ_HANDLED
        } else {
            bindings::irqreturn_IRQ_NONE
        }
    }

    /// # Safety
    ///
    /// `port` is live, `new` is exclusively writable and `old` is null or
    /// readable. Serial core holds the TTY mutex, but not the UART spinlock.
    unsafe extern "C" fn set_termios(
        port: *mut bindings::uart_port,
        new: *mut bindings::ktermios,
        old: *const bindings::ktermios,
    ) {
        // SAFETY: serial core provides valid termios pointers (old may be null)
        // and serializes termios changes. All port data is accessed under lock.
        unsafe {
            let this = Self::from_port(port);
            (*new).c_cflag &= !(bindings::CRTSCTS | bindings::CMSPAR);
            let clock = (*port).uartclk;
            let baud = bindings::uart_get_baud_rate(
                port,
                new,
                old,
                (clock / 16 / 65535).max(50),
                clock / 16,
            );
            let cflag = (*new).c_cflag;
            let iflag = (*new).c_iflag;
            let config = LineConfig {
                baud,
                clock,
                data_bits: match cflag & bindings::CSIZE {
                    bindings::CS5 => 5,
                    bindings::CS6 => 6,
                    bindings::CS7 => 7,
                    _ => 8,
                },
                two_stop_bits: cflag & bindings::CSTOPB != 0,
                parity: cflag & bindings::PARENB != 0,
                odd_parity: cflag & bindings::PARODD != 0,
            };
            Self::locked(port, |_| {
                let actual = match this.hardware.configure(config) {
                    Ok(rate) if rate != 0 => rate,
                    _ => {
                        if !old.is_null() {
                            *new = *old;
                        }
                        return;
                    }
                };
                (*port).read_status_mask = receive::OVERRUN | receive::DATA;
                if iflag & bindings::INPCK != 0 {
                    (*port).read_status_mask |= receive::PARITY | receive::FRAME;
                }
                if iflag & (bindings::IGNBRK | bindings::BRKINT | bindings::PARMRK) != 0 {
                    (*port).read_status_mask |= receive::BREAK;
                }
                (*port).ignore_status_mask = 0;
                if iflag & bindings::IGNPAR != 0 {
                    (*port).ignore_status_mask |= receive::PARITY | receive::FRAME;
                }
                if iflag & bindings::IGNBRK != 0 {
                    (*port).ignore_status_mask |= receive::BREAK;
                    if iflag & bindings::IGNPAR != 0 {
                        (*port).ignore_status_mask |= receive::OVERRUN;
                    }
                }
                if cflag & bindings::CREAD == 0 {
                    (*port).ignore_status_mask |= receive::DATA;
                }
                bindings::uart_update_timeout(port, cflag, actual);
                if (*new).c_ospeed != 0 {
                    bindings::tty_termios_encode_baud_rate(new, actual, actual);
                }
            });
        }
    }

    extern "C" fn port_type(_port: *mut bindings::uart_port) -> *const c_char {
        c"Rust DesignWare UART".as_char_ptr()
    }
    extern "C" fn request_port(_port: *mut bindings::uart_port) -> i32 {
        0
    }
    extern "C" fn release_port(_port: *mut bindings::uart_port) {}
    extern "C" fn config_port(_port: *mut bindings::uart_port, _flags: i32) {}
    extern "C" fn verify_port(
        _port: *mut bindings::uart_port,
        _info: *mut bindings::serial_struct,
    ) -> i32 {
        EINVAL.to_errno()
    }

    const OPS: bindings::uart_ops = bindings::uart_ops {
        tx_empty: Some(Self::tx_empty),
        set_mctrl: Some(Self::set_mctrl),
        get_mctrl: Some(Self::get_mctrl),
        stop_tx: Some(Self::stop_tx),
        flush_buffer: Some(Self::flush_buffer),
        start_tx: Some(Self::start_tx),
        stop_rx: Some(Self::stop_rx),
        break_ctl: Some(Self::break_ctl),
        startup: Some(Self::startup),
        shutdown: Some(Self::shutdown),
        set_termios: Some(Self::set_termios),
        type_: Some(Self::port_type),
        request_port: Some(Self::request_port),
        release_port: Some(Self::release_port),
        config_port: Some(Self::config_port),
        verify_port: Some(Self::verify_port),
        ..pin_init::zeroed()
    };
}

#[pinned_drop]
impl<T: Hardware> PinnedDrop for Registration<'_, T> {
    fn drop(self: Pin<&mut Self>) {
        if self.registered {
            // SAFETY: Both objects were registered by new(). Removing the port
            // hangs up users and synchronizes shutdown/IRQ before hardware drops.
            unsafe {
                bindings::uart_remove_one_port(self.driver.get(), self.port.get());
                bindings::uart_unregister_driver(self.driver.get());
            }
        }
    }
}
