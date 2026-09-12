// SPDX-License-Identifier: GPL-2.0-only

//! Minimal interrupt-driven UART integration with serial core.
//!
//! A [`Driver`] is a module-lifetime `uart_driver`: it owns the TTY major and
//! the per-line state that open TTYs keep referring to until they are closed.
//! A [`Port`] is one device-lifetime line on such a driver. The split follows
//! serial core's ownership rules: unbinding a device only hangs its line up,
//! while an open TTY pins the module so the line state cannot be freed under
//! it. The abstraction deliberately exposes no raw `uart_port` pointer to
//! controller drivers.

use crate::{
    bindings,
    device::{
        Bound,
        Device, //
    },
    error::to_result,
    irq::IrqRequest,
    prelude::*,
    sync::atomic::{
        Acquire,
        Atomic,
        Full,
        Relaxed,
        Release, //
    },
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
    /// Input clock rate in Hz, as returned by [`Hardware::prepare_clock`].
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

/// Controller operations, called with the UART spinlock held unless noted.
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
    /// Prepares the input clock for `baud` and returns the rate it then runs at.
    ///
    /// Unlike the other methods this is called in process context without the
    /// port lock and may sleep. `clock` is the rate currently recorded for the
    /// port; controllers with a fixed clock return it unchanged.
    fn prepare_clock(&self, baud: u32, clock: u32) -> u32 {
        let _ = baud;
        clock
    }
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

/// Static description of a TTY driver.
pub struct DriverInfo {
    /// Name shown under `/proc/tty/driver/`.
    pub driver_name: &'static CStr,
    /// Device node prefix; line `n` becomes `/dev/<dev_name><n>`.
    pub dev_name: &'static CStr,
    /// Number of lines, at most [`Driver::MAX_LINES`].
    pub lines: u32,
}

/// A `uart_driver` meant to live in a `static` for the module's lifetime.
///
/// Register it once from module initialization with [`Driver::register`] and
/// keep the returned [`Registration`] alive for as long as any [`Port`] on
/// this driver can exist. Declare the device driver registration *before*
/// the [`Registration`] in the module struct so that devices are unbound,
/// and their ports removed, before the TTY driver goes away.
///
/// # Invariants
///
/// `raw` is only written by the holder of the [`Self::REGISTERING`] state.
/// While `state` is [`Self::REGISTERED`], `raw` is registered with serial
/// core and the TTY driver's owner is the module that created the ports.
pub struct Driver {
    raw: Opaque<bindings::uart_driver>,
    info: DriverInfo,
    /// Bit `n` is set while line `n` belongs to a [`Port`].
    lines: Atomic<u64>,
    /// One of [`Self::UNREGISTERED`], [`Self::REGISTERING`] or [`Self::REGISTERED`].
    state: Atomic<u32>,
}

// SAFETY: `raw` is only mutated by `register()` and `Registration::drop()`,
// which are serialized through `state`; every other access is a shared read
// of a driver that stays registered while it is used.
unsafe impl Sync for Driver {}

impl Driver {
    /// Largest supported line count; one bit of `lines` per line.
    pub const MAX_LINES: u32 = 64;

    const UNREGISTERED: u32 = 0;
    const REGISTERING: u32 = 1;
    const REGISTERED: u32 = 2;

    /// Creates an unregistered driver.
    pub const fn new(info: DriverInfo) -> Self {
        Self {
            raw: Opaque::zeroed(),
            info,
            lines: Atomic::new(0),
            state: Atomic::new(Self::UNREGISTERED),
        }
    }

    /// Registers the driver with serial core on behalf of `module`.
    ///
    /// An open TTY on any of its lines holds a reference to `module`, so the
    /// module cannot be removed while line state is still reachable.
    pub fn register(&'static self, module: &'static ThisModule) -> Result<Registration> {
        if self.info.lines == 0 || self.info.lines > Self::MAX_LINES {
            return Err(EINVAL);
        }
        if self
            .state
            .cmpxchg(Self::UNREGISTERED, Self::REGISTERING, Full)
            .is_err()
        {
            return Err(EBUSY);
        }
        let raw = self.raw.get();
        // SAFETY: This thread holds the `REGISTERING` state, so nothing else
        // accesses `raw`; the names have static lifetime.
        unsafe {
            raw.write(pin_init::zeroed());
            (*raw).owner = module.as_ptr();
            (*raw).driver_name = self.info.driver_name.as_char_ptr();
            (*raw).dev_name = self.info.dev_name.as_char_ptr();
            (*raw).nr = self.info.lines as i32;
        }
        // SAFETY: `raw` is initialized and, living in a `static`, stays pinned.
        if let Err(err) = to_result(unsafe { bindings::uart_register_driver(raw) }) {
            self.state.store(Self::UNREGISTERED, Release);
            return Err(err);
        }
        // SAFETY: Registration created `tty_driver`; no line can be opened
        // before a `Port` is added, so nothing observes the owner changing.
        unsafe { (*(*raw).tty_driver).owner = module.as_ptr() };
        self.state.store(Self::REGISTERED, Release);
        Ok(Registration { driver: self })
    }

    /// The static description this driver was created with.
    pub fn info(&self) -> &DriverInfo {
        &self.info
    }
}

/// Keeps a [`Driver`] registered; unregisters it when dropped.
pub struct Registration {
    driver: &'static Driver,
}

impl Registration {
    /// The registered driver.
    pub fn driver(&self) -> &'static Driver {
        self.driver
    }
}

impl Drop for Registration {
    fn drop(&mut self) {
        // Ports are removed by their device drivers, which must be torn down
        // first. Leaking the registration is the only memory-safe response if
        // a module got that order wrong: serial core would free line state
        // that live ports still use.
        if self.driver.lines.load(Acquire) != 0 {
            pr_err!(
                "{}: lines still in use at unregistration; leaking the TTY driver\n",
                self.driver.info.driver_name
            );
            return;
        }
        // SAFETY: `register()` succeeded and no `Port` refers to this driver.
        unsafe { bindings::uart_unregister_driver(self.driver.raw.get()) };
        self.driver.state.store(Driver::UNREGISTERED, Release);
    }
}

/// Ownership of one line index of a [`Driver`].
struct Line {
    driver: &'static Driver,
    index: u32,
}

impl Line {
    fn allocate(driver: &'static Driver) -> Result<Self> {
        let mut used = driver.lines.load(Relaxed);
        loop {
            let index = (!used).trailing_zeros();
            if index >= driver.info.lines {
                return Err(EBUSY);
            }
            match driver.lines.cmpxchg(used, used | (1 << index), Full) {
                Ok(_) => return Ok(Self { driver, index }),
                Err(current) => used = current,
            }
        }
    }
}

impl Drop for Line {
    fn drop(&mut self) {
        let mut used = self.driver.lines.load(Relaxed);
        while let Err(current) = self
            .driver
            .lines
            .cmpxchg(used, used & !(1u64 << self.index), Full)
        {
            used = current;
        }
    }
}

/// One TTY line of a registered [`Driver`], tied to a bound device.
///
/// # Invariants
///
/// While `added` is set the port is registered with serial core and
/// `raw.private_data` points at `hardware`. `hardware` and the parent device
/// outlive all callbacks; removal stops IRQs synchronously.
#[pin_data(PinnedDrop)]
pub struct Port<'a, T: Hardware> {
    #[pin]
    hardware: T,
    line: Line,
    #[pin]
    raw: Opaque<bindings::uart_port>,
    _device: PhantomData<&'a Device<Bound>>,
    added: bool,
}

// SAFETY: Hardware is Send + Sync. All mutable uart_port accesses are protected
// by the serial-core port lock or serialized startup/shutdown/core lifecycle.
unsafe impl<T: Hardware> Send for Port<'_, T> {}
// SAFETY: The same UART lock serializes concurrent callbacks.
unsafe impl<T: Hardware> Sync for Port<'_, T> {}

impl<'a, T: Hardware + 'a> Port<'a, T> {
    /// Adds the lowest free line of `driver` for `dev`.
    ///
    /// The caller must supply MMIO obtained from this device, with interrupt
    /// sources disabled. `driver` must currently be registered, and its
    /// [`Registration`] must outlive the returned port.
    pub fn new(
        driver: &'static Driver,
        dev: &'a Device<Bound>,
        irq: IrqRequest<'a>,
        hardware: T,
        mapbase: u64,
        clock: u32,
        fifo_size: u32,
    ) -> impl PinInit<Self, Error> + 'a {
        pin_init::pin_init_scope(move || {
            if driver.state.load(Acquire) != Driver::REGISTERED {
                return Err(ENODEV);
            }
            let line = Line::allocate(driver)?;
            let index = line.index;
            Ok(try_pin_init!(&this in Self {
                hardware,
                line,
                raw <- Opaque::try_ffi_init(|slot: *mut bindings::uart_port| {
                    // SAFETY: The initializer owns the slot. `hardware` was
                    // initialized above and, like every other stored address,
                    // stays valid until uart_remove_one_port() completes in Drop.
                    unsafe {
                        slot.write(pin_init::zeroed());
                        bindings::__spin_lock_init(
                            ptr::addr_of_mut!((*slot).lock),
                            c"rust-uart-port".as_char_ptr(),
                            crate::static_lock_class!().as_ptr(),
                        );
                        (*slot).dev = dev.as_raw();
                        (*slot).irq = irq.irq();
                        (*slot).line = index;
                        (*slot).mapbase = mapbase;
                        (*slot).uartclk = clock;
                        (*slot).fifosize = fifo_size;
                        (*slot).iotype = bindings::uart_iotype_UPIO_MEM32;
                        (*slot).regshift = 2;
                        (*slot).type_ = bindings::PORT_16550A;
                        (*slot).flags = bindings::UPF_FIXED_PORT | bindings::UPF_FIXED_TYPE;
                        (*slot).ops = &Self::OPS;
                        (*slot).private_data =
                            ptr::addr_of!((*this.as_ptr()).hardware).cast_mut().cast();
                    }
                    Ok::<(), Error>(())
                }),
                _device: PhantomData,
                added: {
                    // SAFETY: `raw` was initialized above; callbacks only reach
                    // `hardware`, which is initialized and pinned as well.
                    let raw = unsafe { (*this.as_ptr()).raw.get() };
                    // SAFETY: The driver is registered and outlives this port.
                    to_result(unsafe { bindings::uart_add_one_port(driver.raw.get(), raw) })?;
                    true
                },
            }))
        })
    }

    /// The line index; the device node is `/dev/<dev_name><line>`.
    pub fn line(&self) -> u32 {
        self.line.index
    }

    /// # Safety
    ///
    /// `port` must have been created by [`Port::new`] with this `T`, and that
    /// port must stay registered for the entire returned borrow `'b`.
    unsafe fn hardware<'b>(port: *mut bindings::uart_port) -> &'b T {
        // SAFETY: `private_data` points at the pinned `hardware` of a live port.
        unsafe { &*(*port).private_data.cast::<T>() }
    }

    fn locked<R>(port: *mut bindings::uart_port, f: impl FnOnce(&mut LockedPort<'_>) -> R) -> R {
        let mut flags = 0;
        // SAFETY: All callers pass a live port created by `Port::new`.
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
    /// `port` must belong to a live port; its lock must not be held.
    unsafe extern "C" fn tx_empty(port: *mut bindings::uart_port) -> u32 {
        // SAFETY: serial core only invokes callbacks while the port is registered.
        let hardware = unsafe { Self::hardware(port) };
        Self::locked(port, |_| {
            if hardware.tx_empty() {
                bindings::TIOCSER_TEMT
            } else {
                0
            }
        })
    }

    /// # Safety
    ///
    /// `port` must belong to a live port and its IRQ-safe lock is held.
    unsafe extern "C" fn set_mctrl(port: *mut bindings::uart_port, ctrl: u32) {
        // SAFETY: serial core calls this with the port lock held.
        unsafe { Self::hardware(port) }.set_loopback(ctrl & bindings::TIOCM_LOOP != 0);
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
        unsafe { Self::hardware(port) }.start_tx(&mut LockedPort {
            raw: port,
            _lifetime: PhantomData,
        });
    }

    /// # Safety
    ///
    /// `port` belongs to a live port and its IRQ-safe lock is held.
    unsafe extern "C" fn stop_tx(port: *mut bindings::uart_port) {
        // SAFETY: serial core supplies the port lock.
        unsafe { Self::hardware(port) }.stop_tx();
    }

    /// # Safety
    ///
    /// `port` belongs to a live port and its IRQ-safe lock is held.
    unsafe extern "C" fn flush_buffer(port: *mut bindings::uart_port) {
        // SAFETY: Serial core invokes this with the live port locked.
        unsafe { Self::hardware(port) }.flush_tx();
    }

    /// # Safety
    ///
    /// `port` belongs to a live port and its IRQ-safe lock is held.
    unsafe extern "C" fn stop_rx(port: *mut bindings::uart_port) {
        // SAFETY: serial core supplies the port lock.
        unsafe { Self::hardware(port) }.stop_rx();
    }

    /// # Safety
    ///
    /// `port` is live, its TTY mutex is held, and its spinlock is not held.
    unsafe extern "C" fn break_ctl(port: *mut bindings::uart_port, enabled: i32) {
        // SAFETY: The registered port stays alive during this callback.
        let hardware = unsafe { Self::hardware(port) };
        Self::locked(port, |_| hardware.set_break(enabled != 0));
    }

    /// # Safety
    ///
    /// `port` and its state are live. Serial core serializes this call against
    /// shutdown and calls it only while no IRQ is registered for this port.
    unsafe extern "C" fn startup(port: *mut bindings::uart_port) -> i32 {
        // SAFETY: serial core serializes startup and shutdown; state is live.
        let hardware = unsafe { Self::hardware(port) };
        // SAFETY: The port, its serial-core allocated name and the callback
        // cookie remain live until shutdown's free_irq() synchronizes the
        // handler. Hardware IRQ sources are masked.
        let result = unsafe {
            bindings::request_irq(
                (*port).irq,
                Some(Self::interrupt),
                0,
                (*port).name,
                port.cast(),
            )
        };
        if result < 0 {
            return result;
        }
        if let Err(err) = Self::locked(port, |_| hardware.startup()) {
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
        let hardware = unsafe { Self::hardware(port) };
        Self::locked(port, |_| hardware.shutdown());
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
        let hardware = unsafe { Self::hardware(port) };
        if Self::locked(port, |locked| hardware.interrupt(locked)) {
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
            let hardware = Self::hardware(port);
            (*new).c_cflag &= !(bindings::CRTSCTS | bindings::CMSPAR);
            // Let the controller pick an input clock for the requested rate
            // before serial core clamps the rate to what that clock can divide.
            // B0 hangs up and is treated as 9600 by serial core.
            let requested = match bindings::tty_termios_baud_rate(new) {
                0 => 9600,
                rate => rate,
            };
            let mut clock = hardware.prepare_clock(requested, (*port).uartclk);
            let baud = bindings::uart_get_baud_rate(
                port,
                new,
                old,
                (clock / 16 / 65535).max(50),
                clock / 16,
            );
            if baud != requested {
                clock = hardware.prepare_clock(baud, clock);
            }
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
            let applied = Self::locked(port, |_| {
                let actual = match hardware.configure(config) {
                    Ok(rate) if rate != 0 => rate,
                    _ => return false,
                };
                (*port).uartclk = clock;
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
                true
            });
            if applied {
                return;
            }
            let dev: &Device = Device::from_raw((*port).dev);
            if old.is_null() {
                dev_warn!(
                    dev,
                    "{} baud not applied; the line keeps its previous settings\n",
                    baud
                );
                return;
            }
            // Report the previous settings and move the clock back to them.
            *new = *old;
            let previous = match bindings::tty_termios_baud_rate(old) {
                0 => 9600,
                rate => rate,
            };
            if hardware.prepare_clock(previous, clock) != (*port).uartclk {
                dev_warn!(
                    dev,
                    "{} baud not applied and the input clock could not be restored\n",
                    baud
                );
            }
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
impl<T: Hardware> PinnedDrop for Port<'_, T> {
    fn drop(self: Pin<&mut Self>) {
        if self.added {
            // SAFETY: `new()` added this port to `line.driver`, which stays
            // registered while the port exists. Removal hangs up users and
            // synchronizes shutdown/IRQ handling before `hardware` drops.
            unsafe { bindings::uart_remove_one_port(self.line.driver.raw.get(), self.raw.get()) };
        }
    }
}
