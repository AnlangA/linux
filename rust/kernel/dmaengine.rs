// SPDX-License-Identifier: GPL-2.0-only

//! DMAengine slave-channel primitives for kernel abstractions.
//!
//! These crate-private operations require the caller to serialize channel and
//! buffer ownership. Drivers consume them through the serial DMA abstraction.

use crate::{
    bindings,
    device::{Bound, Device},
    dma::Coherent,
    error::{from_err_ptr, to_result},
    prelude::*,
};
use core::ptr::NonNull;

/// One exclusive slave channel and its coherent bounce buffer.
pub(crate) struct Channel {
    raw: NonNull<bindings::dma_chan>,
    buffer: Option<Coherent<[u8]>>,
    direction: bindings::dma_transfer_direction,
}

// SAFETY: Exclusive DMA channels and coherent allocations may be moved between
// tasks. All shared mutation and DMA/CPU ownership operations are unsafe below.
unsafe impl Send for Channel {}

impl Channel {
    /// Requests and configures an 8-bit slave channel from firmware.
    pub(crate) fn request(
        dev: &Device<Bound>,
        name: &CStr,
        fifo: u64,
        size: usize,
        burst: u32,
        receive: bool,
    ) -> Result<Self> {
        // SAFETY: The bound device and C string remain valid during the call.
        let raw =
            from_err_ptr(unsafe { bindings::dma_request_chan(dev.as_raw(), name.as_char_ptr()) })?;
        let direction = if receive {
            bindings::dma_transfer_direction_DMA_DEV_TO_MEM
        } else {
            bindings::dma_transfer_direction_DMA_MEM_TO_DEV
        };
        let mut channel = Self {
            raw: NonNull::new(raw).ok_or(ENODEV)?,
            buffer: None,
            direction,
        };
        // This first implementation relies on PL330's synchronous pause and
        // non-failing termination. Other providers require a separate audit of
        // their cancellation contract before accepting their channels.
        // SAFETY: The exclusive request keeps the provider device available.
        let provider = unsafe { Device::<Bound>::from_raw((*(*raw).device).dev) };
        provider
            .fwnode()
            .ok_or(ENODEV)?
            .property_match_string(c"compatible", c"arm,pl330")?;
        let mut caps: bindings::dma_slave_caps = pin_init::zeroed();
        // SAFETY: The successful request exclusively owns this live channel.
        to_result(unsafe { bindings::dma_get_slave_caps(raw, &mut caps) })?;
        // Require an actual callback-synchronization operation. Otherwise an
        // already dispatched callback could outlive serial-core port state.
        // SAFETY: dma_request_chan keeps the provider and channel available.
        let can_synchronize = unsafe { (*(*raw).device).device_synchronize.is_some() };
        if !caps.cmd_terminate
            || !can_synchronize
            || (receive
                && (!caps.cmd_pause
                    || caps.residue_granularity
                        == bindings::dma_residue_granularity_DMA_RESIDUE_GRANULARITY_DESCRIPTOR))
        {
            return Err(Error::from_errno(-(bindings::EOPNOTSUPP as i32)));
        }
        let mut cfg: bindings::dma_slave_config = pin_init::zeroed();
        cfg.direction = direction;
        cfg.src_addr = fifo;
        cfg.dst_addr = fifo;
        cfg.src_addr_width = bindings::dma_slave_buswidth_DMA_SLAVE_BUSWIDTH_1_BYTE;
        cfg.dst_addr_width = bindings::dma_slave_buswidth_DMA_SLAVE_BUSWIDTH_1_BYTE;
        cfg.src_maxburst = burst;
        cfg.dst_maxburst = burst;
        // SAFETY: The channel is idle and cfg is initialized for its direction.
        to_result(unsafe { bindings::dmaengine_slave_config(raw, &mut cfg) })?;
        // SAFETY: An exclusive channel request holds the DMA provider's resources
        // until dma_release_channel. Allocate with its device, not the UART's
        // DMA mask/address domain. The allocation retains its own device ref.
        let dma_dev = unsafe { Device::<Bound>::from_raw((*(*raw).device).dev) };
        channel.buffer = Some(Coherent::zeroed_slice(dma_dev, size, GFP_KERNEL)?);
        Ok(channel)
    }

    /// # Safety
    /// The channel must be idle, with no device access or other CPU borrow.
    #[expect(
        clippy::mut_from_ref,
        reason = "caller proves exclusive CPU buffer ownership"
    )]
    pub(crate) unsafe fn buffer_mut(&self) -> &mut [u8] {
        // SAFETY: The caller ensures exclusive CPU ownership of the buffer.
        unsafe {
            self.buffer
                .as_ref()
                .expect("configured DMA buffer")
                .as_mut()
        }
    }

    /// # Safety
    /// The transfer must have completed or been paused, and CPU writes excluded.
    pub(crate) unsafe fn buffer(&self) -> &[u8] {
        crate::sync::barrier::dma_rmb();
        // SAFETY: The caller has stopped writes by the DMA engine.
        unsafe {
            self.buffer
                .as_ref()
                .expect("configured DMA buffer")
                .as_ref()
        }
    }

    /// # Safety
    /// No transfer or buffer borrow may exist. The callback and its cookie must
    /// remain live until completion or a successful stop_sync/channel release.
    pub(crate) unsafe fn submit(
        &self,
        len: usize,
        callback: unsafe extern "C" fn(*mut c_void),
        cookie: *mut c_void,
    ) -> Result<i32> {
        let buffer = self.buffer.as_ref().ok_or(ENODEV)?;
        if len == 0 || len > buffer.len() {
            return Err(EINVAL);
        }
        // SAFETY: The coherent allocation belongs to this channel's DMA device
        // and stays owned for the entire transfer. Bounds checked above.
        let desc = unsafe {
            bindings::dmaengine_prep_slave_single(
                self.raw.as_ptr(),
                buffer.dma_handle(),
                len,
                self.direction,
                (bindings::dma_ctrl_flags_DMA_PREP_INTERRUPT
                    | bindings::dma_ctrl_flags_DMA_CTRL_ACK) as c_ulong,
            )
        };
        if desc.is_null() {
            return Err(EBUSY);
        }
        // SAFETY: prep returns an unsubmitted descriptor owned by this caller.
        unsafe {
            (*desc).callback = Some(callback);
            (*desc).callback_param = cookie;
        }
        // SAFETY: All descriptor data is initialized and remains live as required.
        let result = unsafe { bindings::dmaengine_submit(desc) };
        if result < 0 {
            Err(Error::from_errno(result))
        } else {
            Ok(result)
        }
    }

    /// # Safety
    /// A descriptor has been submitted, with its ownership state published.
    pub(crate) unsafe fn issue(&self) {
        crate::sync::barrier::dma_wmb();
        // SAFETY: The caller ensures a pending descriptor exists.
        unsafe { bindings::dma_async_issue_pending(self.raw.as_ptr()) };
    }

    /// # Safety
    /// Caller must serialize this with submit, stop and buffer access.
    pub(crate) unsafe fn status(&self, cookie: i32) -> (bindings::dma_status, usize) {
        let mut state: bindings::dma_tx_state = pin_init::zeroed();
        // SAFETY: The channel is live and state is writable for this call.
        let status =
            unsafe { bindings::dmaengine_tx_status(self.raw.as_ptr(), cookie, &mut state) };
        (status, state.residue as usize)
    }

    /// # Safety
    /// Serialize with other operations; no CPU buffer borrow may be live.
    pub(crate) unsafe fn pause(&self) -> Result {
        // SAFETY: Channel ownership and serialization are guaranteed by caller.
        to_result(unsafe { bindings::dmaengine_pause(self.raw.as_ptr()) })
    }

    /// # Safety
    /// Serialize with submits. Do not reuse any buffer until stop_sync finishes.
    pub(crate) unsafe fn stop_async(&self) -> Result {
        // SAFETY: The caller keeps all resources/callbacks alive until stop_sync.
        to_result(unsafe { bindings::dmaengine_terminate_async(self.raw.as_ptr()) })
    }

    /// # Safety
    /// Process context, no channel/port lock held, and no concurrent submit.
    pub(crate) unsafe fn stop_sync(&self) -> Result {
        // SAFETY: No callback can deadlock against the caller's locks.
        to_result(unsafe { bindings::dmaengine_terminate_sync(self.raw.as_ptr()) })
    }
}

impl Drop for Channel {
    fn drop(&mut self) {
        // SAFETY: The exclusive owner drops in process context without the port
        // lock. Channel release also frees/synchronizes provider resources before
        // the coherent memory is freed, including on partial request failure.
        unsafe {
            let _ = self.stop_sync();
            bindings::dma_release_channel(self.raw.as_ptr());
        }
        drop(self.buffer.take());
    }
}
