// SPDX-License-Identifier: GPL-2.0-only

//! UART DMA state machine. Every state transition holds the UART port lock.
//! A cancelled buffer is not reused until a process-context worker synchronizes
//! its DMA callback. Close cancels that worker, synchronizes both channels, and
//! only then lets serial core retire its port state.

use super::LockedPort;
use crate::{
    bindings,
    device::{Bound, Device},
    dmaengine::Channel,
    fmt,
    prelude::*,
    sync::atomic::{Atomic, Relaxed},
    time::msecs_to_jiffies,
    types::Opaque,
};

/// Bound the software-stop latency: an already issued DMA block is allowed to
/// complete, but stop_tx prevents chaining the next one. PL330 cannot resume a
/// paused TX descriptor, and its SAR residue includes uncommitted FIFO data.
const TX_SIZE: usize = 256;
const RX_SIZE: usize = 512;
/// Covers exact-burst short packets for which the UART FIFO is already empty
/// and therefore produces no receive-timeout interrupt. RTO handles other tails.
const RX_POLL_MS: u32 = 20;

/// Firmware channel settings for an 8-bit UART FIFO.
#[derive(Clone, Copy)]
pub struct Config {
    /// DMA address of RBR/THR, obtained from the device's MMIO resource.
    pub fifo: u64,
    /// Maximum DMA burst in bytes, matching the UART's receive trigger.
    pub burst: u32,
}

/// Driver-wide counters, independent of any particular open file or DMA buffer.
pub struct Stats {
    channels: Atomic<u64>,
    fallback: Atomic<u64>,
    tx_bytes: Atomic<u64>,
    rx_bytes: Atomic<u64>,
    tx_blocks: Atomic<u64>,
    rx_blocks: Atomic<u64>,
    rx_flushes: Atomic<u64>,
    tx_flushes: Atomic<u64>,
    errors: Atomic<u64>,
}

impl Stats {
    pub(super) const fn new() -> Self {
        Self {
            channels: Atomic::new(0),
            fallback: Atomic::new(0),
            tx_bytes: Atomic::new(0),
            rx_bytes: Atomic::new(0),
            tx_blocks: Atomic::new(0),
            rx_blocks: Atomic::new(0),
            rx_flushes: Atomic::new(0),
            tx_flushes: Atomic::new(0),
            errors: Atomic::new(0),
        }
    }

    /// Formats a read-only diagnostic snapshot; counters can advance concurrently.
    pub fn write(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "dma_ports {}\npio_fallbacks {}\ntx_dma_bytes {}\nrx_dma_bytes {}\ntx_dma_blocks {}\nrx_dma_blocks {}\nrx_dma_flushes {}\ntx_dma_flushes {}\ndma_errors {}",
            self.channels.load(Relaxed), self.fallback.load(Relaxed), self.tx_bytes.load(Relaxed),
            self.rx_bytes.load(Relaxed), self.tx_blocks.load(Relaxed), self.rx_blocks.load(Relaxed),
            self.rx_flushes.load(Relaxed), self.tx_flushes.load(Relaxed), self.errors.load(Relaxed))
    }
}

mod state;
use state::{Phase, State};

struct Channels {
    tx: Channel,
    rx: Channel,
}

/// Functions dispatching through the owning serial::Port's concrete Hardware type.
pub(super) struct Callbacks {
    pub tx: unsafe extern "C" fn(*mut c_void),
    pub rx: unsafe extern "C" fn(*mut c_void),
    pub work: unsafe extern "C" fn(*mut bindings::work_struct),
}

#[pin_data(PinnedDrop)]
pub(super) struct StateMachine {
    // Kept separate from State: stop_sync borrows channels outside the port lock,
    // while callbacks may inspect State under the lock to see cancellation.
    channels: Option<Channels>,
    state: Opaque<State>,
    pub(super) port: *mut bindings::uart_port,
    callbacks: Callbacks,
    stats: &'static Stats,
    #[pin]
    work: Opaque<bindings::delayed_work>,
}

// SAFETY: Fields only mutate under the port lock or after worker/DMA quiescence.
// Channel operations have the same external synchronization requirements.
unsafe impl Send for StateMachine {}
// SAFETY: The UART lock serializes every state and buffer ownership transition.
unsafe impl Sync for StateMachine {}

impl StateMachine {
    pub(super) fn new<'a>(
        dev: &'a Device<Bound>,
        config: Option<Config>,
        stats: &'static Stats,
        port: *mut bindings::uart_port,
        callbacks: Callbacks,
    ) -> impl PinInit<Self, Error> + 'a {
        pin_init::pin_init_scope(move || {
            let channels = if let Some(config) = config {
                let acquire = || -> Result<Channels> {
                    let tx =
                        Channel::request(dev, c"tx", config.fifo, TX_SIZE, config.burst, false)?;
                    let rx =
                        Channel::request(dev, c"rx", config.fifo, RX_SIZE, config.burst, true)?;
                    Ok(Channels { tx, rx })
                };
                match acquire() {
                    Ok(channels) => {
                        stats.channels.add(1, Relaxed);
                        Some(channels)
                    }
                    Err(e) if e == EPROBE_DEFER => return Err(e),
                    Err(e) => {
                        stats.fallback.add(1, Relaxed);
                        dev_info!(dev, "DMA unavailable ({:?}), using interrupt PIO\n", e);
                        None
                    }
                }
            } else {
                None
            };
            Ok(try_pin_init!(Self {
                channels,
                state: Opaque::new(State::new()),
                port,
                work <- Opaque::ffi_init(|slot| {
                    // SAFETY: The pinned work slot is exclusively initialized;
                    // close/drop cancel it before its owner can disappear.
                    unsafe { bindings::init_delayed_work(slot, Some(callbacks.work)) };
                }),
                callbacks,
                stats,
            }))
        })
    }

    pub(super) fn available(&self) -> bool {
        self.channels.is_some()
    }

    /// # Safety
    /// Caller holds this state machine's UART lock, after the previous close.
    pub(super) unsafe fn activate(&self) {
        // SAFETY: Caller guarantees exclusive state access and prior quiescence.
        unsafe {
            self.state.get().write(State {
                active: true,
                receive: true,
                ..State::new()
            })
        };
    }

    /// # Safety
    /// Caller holds the UART lock. No new DMA or work may be issued afterwards.
    pub(super) unsafe fn deactivate(&self) {
        // SAFETY: Serialized by the UART lock.
        unsafe {
            (*self.state.get()).active = false;
            (*self.state.get()).receive = false;
        }
    }

    /// Process context; called after deactivate and before freeing UART state.
    pub(super) fn close(&self) {
        // SAFETY: deactivate prevents all requeues; the port lock is not held.
        unsafe { bindings::cancel_delayed_work_sync(self.work.get()) };
        if let Some(channels) = &self.channels {
            // SAFETY: No submit can occur, and callbacks can acquire the UART
            // lock to observe inactive state. Both buffers remain allocated.
            unsafe {
                let _ = channels.tx.stop_sync();
                let _ = channels.rx.stop_sync();
            }
        }
    }

    /// # Safety
    /// Called under the port lock for a live, active state machine.
    unsafe fn schedule(&self, delay: u32) {
        // SAFETY: State is protected by the caller's port lock.
        if unsafe { (*self.state.get()).active } {
            // SAFETY: The initialized, pinned work remains live until close.
            unsafe { bindings::mod_delayed_work(self.work.get(), msecs_to_jiffies(delay)) };
        }
    }

    /// # Safety
    /// UART lock held, with serial-core transmit state live.
    pub(super) unsafe fn tx_start(&self, port: &mut LockedPort<'_>) -> bool {
        let Some(channels) = &self.channels else {
            return false;
        };
        // SAFETY: The port lock excludes callbacks and other state transitions.
        let state = unsafe { &mut *self.state.get() };
        if !state.active {
            return false;
        }
        if state.configuring {
            return true;
        }
        if state.tx != Phase::Idle {
            return true;
        }
        if state.tx_failed || port.tx_priority() {
            return false;
        }
        if port.tx_stopped() || port.tx_pending() == 0 {
            return false;
        }
        // SAFETY: Idle owns the TX allocation; the borrow ends before submit.
        let count = port.peek_tx(unsafe { channels.tx.buffer_mut() });
        if count == 0 {
            return false;
        }
        // SAFETY: The callback's port cookie stays live through close's stop_sync.
        match unsafe {
            channels
                .tx
                .submit(count, self.callbacks.tx, self.port.cast())
        } {
            Ok(cookie) => {
                state.tx_cookie = cookie;
                state.tx_len = count;
                state.tx = Phase::Running;
                self.stats.tx_blocks.add(1, Relaxed);
                // SAFETY: Running state is published before the DMA can finish.
                unsafe { channels.tx.issue() };
                true
            }
            Err(_) => {
                state.tx_failed = true;
                self.stats.errors.add(1, Relaxed);
                false // The TTY queue was only peeked; PIO can send these bytes.
            }
        }
    }

    /// # Safety
    /// UART lock held. DMA callback owns completed output; TTY state is live.
    pub(super) unsafe fn tx_complete(&self, port: &mut LockedPort<'_>) -> bool {
        // SAFETY: State ownership is serialized by the port lock.
        let state = unsafe { &mut *self.state.get() };
        let Some(count) = state.complete_tx() else {
            return false;
        };
        port.advance_tx(count);
        self.stats.tx_bytes.add(count as u64, Relaxed);
        port.wake_writers();
        true
    }

    /// # Safety
    /// UART lock held. serial core has already cleared its transmit FIFO.
    pub(super) unsafe fn tx_flush(&self) {
        let Some(channels) = &self.channels else {
            return;
        };
        // SAFETY: The port lock excludes the completion callback.
        let state = unsafe { &mut *self.state.get() };
        if !state.cancel_tx() {
            return;
        }
        self.stats.tx_flushes.add(1, Relaxed);
        // SAFETY: Buffer stays allocated, and Stopping prevents reuse/advance.
        unsafe {
            let _ = channels.tx.stop_async();
            self.schedule(0);
        }
    }

    /// # Safety
    /// UART lock held. Used in the tx_empty callback as well as normal I/O.
    pub(super) unsafe fn tx_busy(&self) -> bool {
        // SAFETY: The port lock protects this read.
        let state = unsafe { &*self.state.get() };
        state.active && state.tx != Phase::Idle
    }

    /// # Safety
    /// UART lock held, with live receive state. True means DMA owns the FIFO.
    pub(super) unsafe fn rx_start(&self) -> bool {
        let Some(channels) = &self.channels else {
            return false;
        };
        // SAFETY: All phase transitions are under the port lock.
        let state = unsafe { &mut *self.state.get() };
        if !state.active || !state.receive {
            return false;
        }
        if state.rx == Phase::Stopping {
            return !state.rx_cpu_safe;
        }
        if state.rx == Phase::Running {
            return true;
        }
        if state.configuring {
            return false;
        }
        if state.rx_failed {
            return false;
        }
        // SAFETY: Idle buffer, and callback context lives until synchronized close.
        match unsafe {
            channels
                .rx
                .submit(RX_SIZE, self.callbacks.rx, self.port.cast())
        } {
            Ok(cookie) => {
                state.rx = Phase::Running;
                state.rx_cookie = cookie;
                state.rx_cpu_safe = false;
                self.stats.rx_blocks.add(1, Relaxed);
                // SAFETY: State was published and no buffer borrow exists.
                unsafe {
                    channels.rx.issue();
                    self.schedule(RX_POLL_MS);
                }
                true
            }
            Err(_) => {
                state.rx_failed = true;
                self.stats.errors.add(1, Relaxed);
                false
            }
        }
    }

    /// # Safety
    /// UART lock held. A provider callback guarantees the RX buffer is complete.
    pub(super) unsafe fn rx_complete(&self, port: &mut LockedPort<'_>) -> bool {
        let Some(channels) = &self.channels else {
            return false;
        };
        // SAFETY: The port lock excludes timeout/close and subsequent submits.
        let state = unsafe { &mut *self.state.get() };
        if !state.complete_rx() {
            return false;
        }
        // SAFETY: Completion ended DMA ownership; there is no new descriptor yet.
        port.receive_dma(unsafe { channels.rx.buffer() });
        self.stats.rx_bytes.add(RX_SIZE as u64, Relaxed);
        true
    }

    /// # Safety
    /// UART lock held. Pauses before reading coherent data and returns whether
    /// PIO may safely drain the remaining hardware FIFO.
    pub(super) unsafe fn rx_flush(&self, port: &mut LockedPort<'_>, schedule: bool) -> bool {
        let Some(channels) = &self.channels else {
            return true;
        };
        // SAFETY: Serialized with DMA completion and work by the UART lock.
        let state = unsafe { &mut *self.state.get() };
        if state.rx == Phase::Idle {
            return true;
        }
        if state.rx == Phase::Stopping {
            return state.rx_cpu_safe;
        }
        state.rx = Phase::Stopping;
        // SAFETY: No CPU buffer access is active. Successful pause freezes writes.
        if unsafe { channels.rx.pause() }.is_ok() {
            // SAFETY: Paused DMA has stable, provider-reported residue.
            let (_, residue) = unsafe { channels.rx.status(state.rx_cookie) };
            if let Some(count) = RX_SIZE.checked_sub(residue) {
                // SAFETY: Pause established CPU ownership of the written prefix.
                port.receive_dma(&unsafe { channels.rx.buffer() }[..count]);
                self.stats.rx_bytes.add(count as u64, Relaxed);
                state.rx_cpu_safe = true;
            } else {
                state.rx_failed = true;
                self.stats.errors.add(1, Relaxed);
            }
        } else {
            state.rx_failed = true;
            self.stats.errors.add(1, Relaxed);
        }
        self.stats.rx_flushes.add(1, Relaxed);
        // SAFETY: Stopping forbids reuse until worker stop_sync completes.
        unsafe {
            let _ = channels.rx.stop_async();
        }
        let safe = state.rx_cpu_safe;
        if schedule {
            // SAFETY: The caller holds the lock and close revokes scheduling.
            unsafe {
                self.schedule(0);
            }
        }
        safe
    }

    /// # Safety
    /// UART lock held, before final shutdown or a receive stop request.
    pub(super) unsafe fn stop_receive(&self, port: &mut LockedPort<'_>) {
        // SAFETY: Serialized with callbacks.
        unsafe {
            (*self.state.get()).receive = false;
            self.rx_flush(port, true);
        }
    }

    /// # Safety
    /// UART lock held; worker and callbacks share the same port context.
    pub(super) unsafe fn work_prepare(&self, port: &mut LockedPort<'_>) -> (bool, bool) {
        // SAFETY: Caller holds the UART lock.
        if !unsafe { (*self.state.get()).active } {
            return (false, false);
        }
        // Flush even exact-burst short messages with no hardware RTO.
        // SAFETY: The caller holds the lock; this worker performs synchronization.
        unsafe {
            self.rx_flush(port, false);
        }
        // SAFETY: Protected phase snapshot.
        let state = unsafe { &*self.state.get() };
        (state.tx == Phase::Stopping, state.rx == Phase::Stopping)
    }

    /// Worker context, no UART lock. Close waits for this worker before freeing.
    pub(super) fn synchronize(&self, tx: bool, rx: bool) {
        if let Some(channels) = &self.channels {
            // SAFETY: Stopping prevents submits. Callback context and memory live
            // until the worker is cancelled/synchronized by close.
            unsafe {
                if tx {
                    let _ = channels.tx.stop_sync();
                }
                if rx {
                    let _ = channels.rx.stop_sync();
                }
            }
        }
    }

    /// # Safety
    /// UART lock held; synchronize has completed for exactly these directions.
    pub(super) unsafe fn work_finish(&self, tx: bool, rx: bool) -> bool {
        // SAFETY: The caller holds the UART lock.
        let state = unsafe { &mut *self.state.get() };
        if tx {
            state.tx = Phase::Idle;
        }
        if rx {
            state.rx = Phase::Idle;
            state.rx_cpu_safe = true;
        }
        state.active && !state.configuring
    }

    /// # Safety
    /// UART lock held, in a termios operation which pins serial-core state.
    pub(super) unsafe fn configure_begin(&self, port: &mut LockedPort<'_>) {
        // SAFETY: The port lock excludes other state transitions.
        unsafe {
            (*self.state.get()).configuring = true;
            self.rx_flush(port, true);
        }
    }

    /// # Safety
    /// UART lock held; used to await the last bounded TX block without cancelling it.
    pub(super) unsafe fn tx_running(&self) -> bool {
        // SAFETY: State is protected by the port lock.
        unsafe { (*self.state.get()).tx == Phase::Running }
    }

    /// Process context with configuring set, after the last TX DMA block completed.
    pub(super) fn configure_sync(&self) {
        // SAFETY: Configuring prevents new submissions; close cannot run while
        // the termios operation holds the TTY mutex.
        unsafe { bindings::cancel_delayed_work_sync(self.work.get()) };
        self.synchronize(true, true);
    }

    /// # Safety
    /// UART lock held. If synchronized, both channels and callbacks are quiescent.
    pub(super) unsafe fn configure_end(&self, synchronized: bool) -> bool {
        // SAFETY: The caller holds the UART port lock.
        let state = unsafe { &mut *self.state.get() };
        if synchronized {
            state.tx = Phase::Idle;
            state.rx = Phase::Idle;
            state.rx_cpu_safe = true;
        }
        state.configuring = false;
        state.active
    }

    /// # Safety
    /// UART lock held after a receive line-status error. DMA cannot provide a
    /// per-byte error sideband, so use PIO for the rest of this open session.
    pub(super) unsafe fn rx_error(&self, port: &mut LockedPort<'_>) -> bool {
        // SAFETY: State is serialized; rx_flush establishes CPU ownership.
        unsafe {
            (*self.state.get()).rx_failed = true;
        }
        if self.channels.is_some() {
            self.stats.errors.add(1, Relaxed);
        }
        // SAFETY: The caller holds the port lock with live TTY state.
        unsafe { self.rx_flush(port, true) }
    }

    /// # Safety
    /// `work` must be the embedded work item of a live StateMachine.
    pub(super) unsafe fn from_work<'a>(work: *mut bindings::work_struct) -> &'a Self {
        // delayed_work begins with work_struct, and Opaque is transparent.
        // SAFETY: The work callback receives the exact pinned item initialized above.
        unsafe {
            &*work
                .cast::<u8>()
                .sub(core::mem::offset_of!(Self, work))
                .cast::<Self>()
        }
    }
}

#[pinned_drop]
impl PinnedDrop for StateMachine {
    fn drop(self: Pin<&mut Self>) {
        // Port teardown has stopped IRQ and callbacks; failed probe never starts
        // DMA. Keep this defensive cancellation before channel/buffer destructors.
        self.close();
        if self.channels.is_some() {
            self.stats.channels.fetch_sub(1, Relaxed);
        }
    }
}
