// SPDX-License-Identifier: GPL-2.0-only

//! A bounded, per-open byte FIFO for character-device regression tests.
//!
//! Each open has an independent FIFO; dup/fork share their existing open file.
//! read/write and poll follow stream semantics and never allocate in I/O paths.

use kernel::{
    fs::{
        file::flags::O_NONBLOCK,
        File,
        Kiocb, //
    },
    iov::{
        IovIterDest,
        IovIterSource, //
    },
    miscdevice::{
        self,
        MiscDevice,
        MiscDeviceOptions,
        MiscDeviceRegistration, //
    },
    new_mutex, new_poll_condvar,
    prelude::*,
    sync::{
        poll::{
            PollCondVar,
            PollTable, //
        },
        Arc,
        ArcBorrow,
        Mutex, //
    },
};

#[path = "rust_chardev/ring.rs"]
mod ring;

const CAPACITY: usize = 4096;

module! {
    type: RustCharModule,
    name: "rust_chardev",
    authors: ["ATK Rust driver contributors"],
    description: "Bounded Rust character-device FIFO",
    license: "GPL v2",
}

#[pin_data]
struct RustCharModule {
    #[pin]
    _registration: MiscDeviceRegistration<OpenFile>,
}

impl kernel::InPlaceModule for RustCharModule {
    fn init(_module: &'static ThisModule) -> impl PinInit<Self, Error> {
        let options = MiscDeviceOptions {
            name: c"rust-chardev",
        };
        try_pin_init!(Self {
            _registration <- MiscDeviceRegistration::register(options),
        })
    }
}

struct Buffer {
    data: KVec<u8>,
    ring: ring::Ring,
}

#[pin_data]
struct OpenFile {
    #[pin]
    buffer: Mutex<Buffer>,
    #[pin]
    changed: PollCondVar,
}

#[vtable]
impl MiscDevice for OpenFile {
    const OWNER: &'static ThisModule = &THIS_MODULE;
    const STREAM: bool = true;
    type Ptr = Arc<Self>;

    fn open(_file: &File, _misc: &MiscDeviceRegistration<Self>) -> Result<Arc<Self>> {
        let mut data = KVec::with_capacity(CAPACITY, GFP_KERNEL_ACCOUNT)?;
        data.resize(CAPACITY, 0, GFP_KERNEL_ACCOUNT)?;
        Arc::pin_init(
            pin_init!(Self {
                buffer <- new_mutex!(Buffer { data, ring: ring::Ring::new(CAPACITY) }),
                changed <- new_poll_condvar!(),
            }),
            GFP_KERNEL_ACCOUNT,
        )
    }

    fn read_iter(kiocb: Kiocb<'_, Self::Ptr>, iov: &mut IovIterDest<'_>) -> Result<usize> {
        if iov.is_empty() {
            return Ok(0);
        }
        let this = kiocb.file();
        let nonblocking = kiocb.flags() & O_NONBLOCK != 0;
        // The mutex is only held for short copies, so waiting for it does not
        // block in the O_NONBLOCK sense; only an empty FIFO does.
        let mut buffer = this.buffer.lock();
        while buffer.ring.is_empty() {
            if nonblocking {
                return Err(EAGAIN);
            }
            if this.changed.wait_interruptible(&mut buffer) {
                return Err(ERESTARTSYS);
            }
        }
        let range = buffer.ring.readable(iov.len());
        let copied = iov.copy_to_iter(&buffer.data[range]);
        if copied == 0 {
            return Err(EFAULT);
        }
        // Consume only successfully copied bytes; a user fault loses no data.
        buffer.ring.consume(copied);
        drop(buffer);
        this.changed.notify_all();
        Ok(copied)
    }

    fn write_iter(kiocb: Kiocb<'_, Self::Ptr>, iov: &mut IovIterSource<'_>) -> Result<usize> {
        if iov.is_empty() {
            return Ok(0);
        }
        let this = kiocb.file();
        let nonblocking = kiocb.flags() & O_NONBLOCK != 0;
        let mut buffer = this.buffer.lock();
        while buffer.ring.is_full() {
            if nonblocking {
                return Err(EAGAIN);
            }
            if this.changed.wait_interruptible(&mut buffer) {
                return Err(ERESTARTSYS);
            }
        }
        let range = buffer.ring.writable(iov.len());
        let copied = iov.copy_from_iter(&mut buffer.data[range]);
        if copied == 0 {
            return Err(EFAULT);
        }
        buffer.ring.produce(copied);
        drop(buffer);
        this.changed.notify_all();
        Ok(copied)
    }

    fn poll(this: ArcBorrow<'_, Self>, file: &File, table: &PollTable<'_>) -> u32 {
        // Register before checking the predicate to avoid missed wakeups.
        table.register_wait(file, &this.changed);
        let buffer = this.buffer.lock();
        let mut mask = 0;
        if !buffer.ring.is_empty() {
            mask |= miscdevice::POLLIN;
        }
        if !buffer.ring.is_full() {
            mask |= miscdevice::POLLOUT;
        }
        mask
    }
}
