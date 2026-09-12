// SPDX-License-Identifier: GPL-2.0-only

use rs485_test::*;
use std::{
    collections::VecDeque,
    io::{self, Read, Write},
    os::{
        fd::{AsRawFd, FromRawFd, OwnedFd},
        unix::fs::FileExt,
    },
    time::{Duration, Instant},
};

// Linux UAPI asm-generic/termios.h; libc does not currently export this flag.
const TIOCM_LOOP: libc::c_int = 0x8000;

const USAGE: &str = "Usage:
  rs485-test uart --device /dev/ttyRU0 [--baud 115200] [--count 100]
      [--size 256] [--timeout-ms 3000]
  rs485-test loopback --device /dev/ttyRU0 [--baud 115200] [--count 100] [--size 256]
  rs485-test peer --device /dev/ttyUSB0 [--baud 115200] [--count 100] [--timeout-ms 30000]
  rs485-test chardev [--device /dev/rust-chardev]

uart: send numbered, CRC-protected frames and require exact echoes.
loopback: test the UART controller's internal loopback, without an external peer.
peer: validate and echo frames through a second RS485 adapter (half duplex).
chardev: validate FIFO bounds, nonblocking I/O, poll, faults, and per-open isolation.
Start the peer before the uart test; both ends must use identical baud/count.
No RTS/DE toggling is needed for the ATK-DLRK3588B automatic-direction circuit.";

struct Options {
    command: String,
    device: String,
    baud: u32,
    count: u32,
    size: usize,
    timeout: Duration,
}

fn options() -> io::Result<Option<Options>> {
    let mut args = std::env::args().skip(1);
    let Some(command) = args.next() else {
        println!("{USAGE}");
        return Ok(None);
    };
    if command == "--help" || command == "-h" {
        println!("{USAGE}");
        return Ok(None);
    }
    if !["uart", "loopback", "peer", "chardev"].contains(&command.as_str()) {
        return Err(invalid("unknown command; use --help"));
    }
    let mut opts = Options {
        device: if command == "chardev" {
            "/dev/rust-chardev".into()
        } else {
            String::new()
        },
        timeout: Duration::from_millis(if command == "peer" { 30000 } else { 3000 }),
        command,
        baud: 115200,
        count: 100,
        size: 256,
    };
    let mut seen = std::collections::HashSet::new();
    while let Some(flag) = args.next() {
        if !seen.insert(flag.clone()) {
            return Err(invalid("duplicate option"));
        }
        let value = args
            .next()
            .ok_or_else(|| invalid("option requires a value"))?;
        match flag.as_str() {
            "--device" => opts.device = value,
            "--baud" => opts.baud = value.parse().map_err(|_| invalid("invalid baud"))?,
            "--count" => opts.count = value.parse().map_err(|_| invalid("invalid count"))?,
            "--size" => opts.size = value.parse().map_err(|_| invalid("invalid size"))?,
            "--timeout-ms" => {
                let millis: u64 = value.parse().map_err(|_| invalid("invalid timeout"))?;
                if !(1..=600000).contains(&millis) {
                    return Err(invalid("timeout must be 1..600000 ms"));
                }
                opts.timeout = Duration::from_millis(millis);
            }
            _ => return Err(invalid("unknown option; use --help")),
        }
    }
    if opts.device.is_empty() {
        return Err(invalid("--device is required"));
    }
    if !(1..=1_000_000).contains(&opts.count) {
        return Err(invalid("count must be 1..1000000"));
    }
    if opts.size > MAX_PAYLOAD {
        return Err(invalid("size must be 0..1024"));
    }
    Ok(Some(opts))
}

fn uart(opts: &Options) -> io::Result<()> {
    let mut port = Serial::open(&opts.device, opts.baud)?;
    let _loopback = if opts.command == "loopback" {
        Some(Loopback::enable(&port.file)?)
    } else {
        None
    };
    let start = Instant::now();
    for sequence in 0..opts.count {
        let payload: Vec<_> = (0..opts.size)
            .map(|i| (sequence ^ i as u32 ^ (i as u32 >> 8)) as u8)
            .collect();
        let frame = encode(sequence, &payload)?;
        if opts.command == "uart" {
            turnaround(opts.baud);
        }
        let end = deadline(opts.timeout)?;
        write_deadline(&mut port.file, &frame, end)?;
        let response = read_frame(&mut port.file, end).map_err(|error| {
            io::Error::new(error.kind(), format!("sequence {sequence}: {error}"))
        })?;
        if response != frame {
            return Err(invalid(&format!("echo mismatch at sequence {sequence}")));
        }
    }
    println!(
        "PASS: {} frames, {} payload bytes/frame, {:.3} s, device {}",
        opts.count,
        opts.size,
        start.elapsed().as_secs_f64(),
        opts.device
    );
    Ok(())
}

struct Loopback {
    file: std::fs::File,
    was_enabled: bool,
}

impl Loopback {
    fn enable(file: &std::fs::File) -> io::Result<Self> {
        let file = file.try_clone()?;
        let mut previous: libc::c_int = 0;
        // SAFETY: TIOCMGET writes one c_int into the supplied valid buffer.
        if unsafe { libc::ioctl(file.as_raw_fd(), libc::TIOCMGET, &mut previous) } < 0 {
            return Err(io::Error::last_os_error());
        }
        let mask = TIOCM_LOOP;
        // SAFETY: TIOCMBIS reads one c_int. Only internal loopback is requested.
        if unsafe { libc::ioctl(file.as_raw_fd(), libc::TIOCMBIS, &mask) } < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(Self {
            file,
            was_enabled: previous & mask != 0,
        })
    }
}

impl Drop for Loopback {
    fn drop(&mut self) {
        if !self.was_enabled {
            let mask = TIOCM_LOOP;
            // SAFETY: The descriptor and mask outlive this synchronous ioctl.
            let _ = unsafe { libc::ioctl(self.file.as_raw_fd(), libc::TIOCMBIC, &mask) };
        }
    }
}

fn peer(opts: &Options) -> io::Result<()> {
    let mut port = Serial::open(&opts.device, opts.baud)?;
    let mut last_frame_bytes = 0;
    println!(
        "Ready: {} at {} baud; expecting {} frames",
        opts.device, opts.baud, opts.count
    );
    for expected_sequence in 0..opts.count {
        let frame = read_frame(&mut port.file, deadline(opts.timeout)?)?;
        let (sequence, _) = decode(&frame)?;
        if sequence != expected_sequence {
            return Err(invalid("missing, duplicated, or reordered request"));
        }
        // Leave a bus-idle interval for both transceivers to release their drivers.
        turnaround(opts.baud);
        write_deadline(&mut port.file, &frame, deadline(opts.timeout)?)?;
        last_frame_bytes = frame.len();
    }
    port.drain()?;
    // USB serial drivers such as ch341 lack a hardware tx_empty callback:
    // tcdrain can finish once USB transfers complete, before the chip's FIFO
    // reaches the wire. This stop-and-wait protocol has at most one response
    // outstanding. Allow that entire final frame's 8N1 wire time, plus 3% baud
    // tolerance and 2 ms USB scheduling margin, before restoring baud settings.
    let micros = ((last_frame_bytes as u64 + 2) * 10_000_000 * 103)
        .div_ceil(u64::from(opts.baud) * 100)
        + 2000;
    std::thread::sleep(Duration::from_micros(micros));
    println!("PASS: echoed {} validated frames", opts.count);
    Ok(())
}

/// Test-protocol turnaround: at least four 8N1 characters, or 2 ms at high baud.
/// Receiving the last byte does not imply the other transceiver has released DE.
fn turnaround(baud: u32) {
    let micros = 40_000_000u64.div_ceil(u64::from(baud)).max(2000);
    std::thread::sleep(Duration::from_micros(micros));
}

fn require(condition: bool, message: &str) -> io::Result<()> {
    if condition {
        Ok(())
    } else {
        Err(io::Error::other(message))
    }
}

fn ready_now(fd: i32, events: i16) -> io::Result<bool> {
    let mut pfd = libc::pollfd {
        fd,
        events,
        revents: 0,
    };
    // SAFETY: pfd points to one initialized pollfd.
    if unsafe { libc::poll(&mut pfd, 1, 0) } < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(pfd.revents & events != 0)
}

fn chardev(opts: &Options) -> io::Result<()> {
    let mut file = open_nonblocking(&opts.device)?;
    let mut byte = [0u8; 1];
    require(
        matches!(file.read(&mut byte), Err(e) if e.kind() == io::ErrorKind::WouldBlock),
        "empty FIFO must return EAGAIN",
    )?;
    require(
        !ready_now(file.as_raw_fd(), libc::POLLIN)?,
        "empty FIFO reported readable",
    )?;
    require(
        file.read(&mut [])? == 0 && file.write(&[])? == 0,
        "zero-length I/O failed",
    )?;
    require(
        matches!(file.write_at(b"x", 0), Err(e) if e.raw_os_error() == Some(libc::ESPIPE)),
        "stream accepted positioned I/O",
    )?;

    let mut reference = VecDeque::new();
    let data: Vec<_> = (0..511).map(|i| i as u8).collect();
    loop {
        match file.write(&data) {
            Ok(0) => return Err(io::ErrorKind::WriteZero.into()),
            Ok(count) => {
                reference.extend(&data[..count]);
                require(reference.len() <= 4096, "FIFO exceeded its capacity")?;
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => break,
            Err(error) => return Err(error),
        }
    }
    require(reference.len() == 4096, "FIFO capacity is not 4096")?;
    require(
        !ready_now(file.as_raw_fd(), libc::POLLOUT)?,
        "full FIFO reported writable",
    )?;
    let mut partial = [0u8; 513];
    let count = file.read(&mut partial)?;
    for actual in &partial[..count] {
        require(
            reference.pop_front() == Some(*actual),
            "data mismatch before wrap",
        )?;
    }
    let count = file.write(&data)?;
    reference.extend(&data[..count]);
    while !reference.is_empty() {
        let count = file.read(&mut partial)?;
        require(count != 0, "unexpected FIFO EOF")?;
        for actual in &partial[..count] {
            require(
                reference.pop_front() == Some(*actual),
                "FIFO wrap corrupted bytes",
            )?;
        }
    }

    // SAFETY: An intentionally invalid userspace buffer is passed to a syscall;
    // the kernel must reject it with EFAULT, without consuming or creating data.
    let result = unsafe { libc::write(file.as_raw_fd(), std::ptr::null(), 1) };
    require(
        result == -1 && io::Error::last_os_error().raw_os_error() == Some(libc::EFAULT),
        "invalid write buffer did not return EFAULT",
    )?;
    file.write_all(b"Z")?;
    // SAFETY: As above, exercising copy_to_user's syscall fault handling.
    let result = unsafe { libc::read(file.as_raw_fd(), std::ptr::null_mut(), 1) };
    require(
        result == -1 && io::Error::last_os_error().raw_os_error() == Some(libc::EFAULT),
        "invalid read buffer did not return EFAULT",
    )?;
    require(
        file.read(&mut byte)? == 1 && byte[0] == b'Z',
        "read fault consumed data",
    )?;
    let mut independent = open_nonblocking(&opts.device)?;
    file.write_all(b"A")?;
    require(
        matches!(independent.read(&mut byte), Err(e) if e.kind() == io::ErrorKind::WouldBlock),
        "independent opens share data",
    )?;
    file.read_exact(&mut byte)?;

    let mut reader = file.try_clone()?;
    let (started_tx, started_rx) = std::sync::mpsc::channel();
    let waiter = std::thread::spawn(move || -> io::Result<()> {
        started_tx.send(()).map_err(io::Error::other)?;
        let mut received = [0u8; 4];
        read_deadline(
            &mut reader,
            &mut received,
            deadline(Duration::from_secs(2))?,
        )?;
        require(received == *b"wake", "poll wakeup payload mismatch")
    });
    started_rx.recv().map_err(io::Error::other)?;
    file.write_all(b"wake")?;
    waiter
        .join()
        .map_err(|_| io::Error::other("poll waiter panicked"))??;

    // SAFETY: epoll_create1 accepts EPOLL_CLOEXEC and returns an owned fd.
    let epfd = unsafe { libc::epoll_create1(libc::EPOLL_CLOEXEC) };
    if epfd < 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: The successful epoll_create1 call transferred this fd to us.
    let epoll = unsafe { OwnedFd::from_raw_fd(epfd) };
    let mut event = libc::epoll_event {
        events: libc::EPOLLIN as u32,
        u64: 1,
    };
    // SAFETY: Both descriptors are live and event points to initialized storage.
    if unsafe {
        libc::epoll_ctl(
            epoll.as_raw_fd(),
            libc::EPOLL_CTL_ADD,
            file.as_raw_fd(),
            &mut event,
        )
    } < 0
    {
        return Err(io::Error::last_os_error());
    }
    file.write_all(b"E")?;
    // SAFETY: event has space for the single requested output event.
    let count = unsafe { libc::epoll_wait(epoll.as_raw_fd(), &mut event, 1, 1000) };
    require(
        count == 1 && event.events & libc::EPOLLIN as u32 != 0,
        "epoll failed to report input",
    )?;
    file.read_exact(&mut byte)?;
    require(byte[0] == b'E', "epoll payload mismatch")?;
    // Close a watched file while the epoll descriptor remains alive.
    drop(file);
    drop(epoll);
    println!(
        "PASS: bounded FIFO, wraparound, zero/partial I/O, EAGAIN, poll/epoll, EFAULT, ESPIPE, and per-open isolation"
    );
    Ok(())
}

fn run() -> io::Result<()> {
    let Some(opts) = options()? else {
        return Ok(());
    };
    match opts.command.as_str() {
        "uart" | "loopback" => uart(&opts),
        "peer" => peer(&opts),
        "chardev" => chardev(&opts),
        _ => unreachable!(),
    }
    .map_err(|error| io::Error::new(error.kind(), format!("{}: {error}", opts.device)))
}

fn main() -> std::process::ExitCode {
    match run() {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("FAIL: {error}");
            std::process::ExitCode::FAILURE
        }
    }
}
