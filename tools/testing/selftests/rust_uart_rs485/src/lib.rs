// SPDX-License-Identifier: GPL-2.0-only

//! Nonblocking, deadline-bounded UART I/O and a small integrity-test protocol.

use std::{
    fs::{File, OpenOptions},
    io::{self, Read, Write},
    os::fd::{AsRawFd, RawFd},
    os::unix::fs::OpenOptionsExt,
    time::{Duration, Instant},
};

pub const MAX_PAYLOAD: usize = 1024;
const MAGIC: [u8; 4] = *b"RU48";

pub fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message)
}

/// IEEE CRC-32, with the standard initial and final inversions.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & 0u32.wrapping_sub(crc & 1));
        }
    }
    !crc
}

pub fn encode(sequence: u32, payload: &[u8]) -> io::Result<Vec<u8>> {
    if payload.len() > MAX_PAYLOAD {
        return Err(invalid("payload exceeds 1024 bytes"));
    }
    let mut frame = Vec::with_capacity(14 + payload.len());
    frame.extend_from_slice(&MAGIC);
    frame.extend_from_slice(&sequence.to_le_bytes());
    frame.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    frame.extend_from_slice(payload);
    frame.extend_from_slice(&crc32(&frame).to_le_bytes());
    Ok(frame)
}

pub fn decode(frame: &[u8]) -> io::Result<(u32, &[u8])> {
    if frame.len() < 14 || frame[..4] != MAGIC {
        return Err(invalid("invalid frame header"));
    }
    let length = u16::from_le_bytes([frame[8], frame[9]]) as usize;
    if length > MAX_PAYLOAD || frame.len() != length + 14 {
        return Err(invalid("invalid frame length"));
    }
    let end = frame.len() - 4;
    let expected = u32::from_le_bytes(frame[end..].try_into().map_err(|_| invalid("CRC length"))?);
    if crc32(&frame[..end]) != expected {
        return Err(invalid("CRC mismatch"));
    }
    let sequence = u32::from_le_bytes(
        frame[4..8]
            .try_into()
            .map_err(|_| invalid("sequence length"))?,
    );
    Ok((sequence, &frame[10..end]))
}

/// Waits without extending the deadline on EINTR or spurious readiness.
pub fn wait_ready(fd: RawFd, events: i16, deadline: Instant) -> io::Result<()> {
    loop {
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .ok_or_else(|| io::Error::new(io::ErrorKind::TimedOut, "I/O deadline expired"))?;
        let millis = remaining
            .as_millis()
            .saturating_add(1)
            .min(i32::MAX as u128) as i32;
        let mut pfd = libc::pollfd {
            fd,
            events,
            revents: 0,
        };
        // SAFETY: pfd points to one initialized, writable pollfd for the call.
        let result = unsafe { libc::poll(&mut pfd, 1, millis) };
        if result < 0 {
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            return Err(error);
        }
        if result == 0 {
            continue;
        }
        if pfd.revents & libc::POLLNVAL != 0 {
            return Err(invalid("invalid descriptor"));
        }
        // Allow draining readable bytes when POLLIN and POLLHUP arrive together.
        if pfd.revents & events != 0 {
            return Ok(());
        }
        if pfd.revents & (libc::POLLHUP | libc::POLLERR) != 0 {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "device disconnected",
            ));
        }
    }
}

pub fn write_deadline(file: &mut File, mut data: &[u8], deadline: Instant) -> io::Result<()> {
    while !data.is_empty() {
        wait_ready(file.as_raw_fd(), libc::POLLOUT, deadline)?;
        match file.write(data) {
            Ok(0) => return Err(io::ErrorKind::WriteZero.into()),
            Ok(count) => data = &data[count..],
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock
                ) => {}
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

pub fn read_deadline(file: &mut File, mut data: &mut [u8], deadline: Instant) -> io::Result<()> {
    while !data.is_empty() {
        wait_ready(file.as_raw_fd(), libc::POLLIN, deadline)?;
        match file.read(data) {
            Ok(0) => return Err(io::ErrorKind::UnexpectedEof.into()),
            Ok(count) => data = &mut data[count..],
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock
                ) => {}
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

pub fn read_frame(file: &mut File, deadline: Instant) -> io::Result<Vec<u8>> {
    let mut header = [0; 10];
    read_deadline(file, &mut header, deadline)?;
    if header[..4] != MAGIC {
        return Err(invalid(&format!(
            "lost frame synchronization: {:02x?}",
            &header[..4]
        )));
    }
    let length = u16::from_le_bytes([header[8], header[9]]) as usize;
    if length > MAX_PAYLOAD {
        return Err(invalid("received oversized frame"));
    }
    let mut frame = vec![0; length + 14];
    frame[..10].copy_from_slice(&header);
    read_deadline(file, &mut frame[10..], deadline)?;
    decode(&frame)?;
    Ok(frame)
}

pub fn open_nonblocking(path: &str) -> io::Result<File> {
    OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC)
        .open(path)
}

/// A UART whose previous termios state is restored before its file is closed.
pub struct Serial {
    pub file: File,
    original: libc::termios,
}

impl Serial {
    /// Drains the kernel queue before changing framing or closing the port.
    pub fn drain(&self) -> io::Result<()> {
        // SAFETY: The descriptor is a live TTY; no user pointers are retained.
        if unsafe { libc::tcdrain(self.file.as_raw_fd()) } < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let speed = match baud {
            9600 => libc::B9600,
            19200 => libc::B19200,
            38400 => libc::B38400,
            57600 => libc::B57600,
            115200 => libc::B115200,
            230400 => libc::B230400,
            460800 => libc::B460800,
            921600 => libc::B921600,
            1500000 => libc::B1500000,
            _ => return Err(invalid("unsupported baud rate")),
        };
        let file = open_nonblocking(path)?;
        // SAFETY: All-zero termios is valid storage, initialized by tcgetattr.
        let mut original: libc::termios = unsafe { std::mem::zeroed() };
        // SAFETY: The descriptor is live and original is a writable termios.
        if unsafe { libc::tcgetattr(file.as_raw_fd(), &mut original) } < 0 {
            return Err(io::Error::last_os_error());
        }
        let serial = Self { file, original };
        let mut raw = original;
        // SAFETY: raw is initialized and uniquely borrowed for these mutations.
        unsafe {
            libc::cfmakeraw(&mut raw);
        }
        raw.c_iflag &= !(libc::IXON | libc::IXOFF | libc::IXANY);
        raw.c_cflag &= !(libc::CSIZE | libc::CSTOPB | libc::PARENB | libc::CMSPAR | libc::CRTSCTS);
        raw.c_cflag |= libc::CS8 | libc::CREAD | libc::CLOCAL;
        raw.c_cc[libc::VMIN] = 1;
        raw.c_cc[libc::VTIME] = 0;
        // SAFETY: A validated speed and initialized termios are passed to libc.
        if unsafe { libc::cfsetispeed(&mut raw, speed) } < 0
            // SAFETY: Same initialized termios and validated speed.
            || unsafe { libc::cfsetospeed(&mut raw, speed) } < 0
            // SAFETY: The file is live and raw remains valid for tcsetattr.
            || unsafe { libc::tcsetattr(serial.file.as_raw_fd(), libc::TCSANOW, &raw) } < 0
        {
            return Err(io::Error::last_os_error());
        }
        let mut applied = original;
        // SAFETY: The live TTY writes its effective settings into applied.
        if unsafe { libc::tcgetattr(serial.file.as_raw_fd(), &mut applied) } < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: applied is an initialized termios returned by tcgetattr.
        if unsafe { libc::cfgetospeed(&applied) } != speed
            || applied.c_cflag & (libc::CSIZE | libc::CSTOPB | libc::PARENB | libc::CRTSCTS)
                != libc::CS8
        {
            return Err(invalid(
                "driver did not accept the requested baud rate and 8N1 framing",
            ));
        }
        // Discard stale input only. Never flush queued outgoing test data.
        // SAFETY: The file is a TTY and TCIFLUSH is a valid selector.
        if unsafe { libc::tcflush(serial.file.as_raw_fd(), libc::TCIFLUSH) } < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(serial)
    }
}

impl Drop for Serial {
    fn drop(&mut self) {
        // SAFETY: The file outlives Drop and original came from tcgetattr.
        let _ = unsafe { libc::tcsetattr(self.file.as_raw_fd(), libc::TCSADRAIN, &self.original) };
    }
}

pub fn deadline(timeout: Duration) -> io::Result<Instant> {
    Instant::now()
        .checked_add(timeout)
        .ok_or_else(|| invalid("timeout is too large"))
}

#[cfg(test)]
#[path = "../../../../../drivers/misc/rust_chardev/ring.rs"]
mod ring_tests;
#[cfg(test)]
#[path = "../../../../../drivers/tty/serial/rust_dw_uart/config.rs"]
mod uart_config_tests;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_known_answer() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn all_payload_lengths_roundtrip() {
        for size in 0..=MAX_PAYLOAD {
            let payload: Vec<_> = (0..size).map(|i| i as u8).collect();
            let frame = encode(u32::MAX, &payload).unwrap();
            assert_eq!(decode(&frame).unwrap(), (u32::MAX, payload.as_slice()));
        }
        assert!(encode(0, &vec![0; MAX_PAYLOAD + 1]).is_err());
    }

    #[test]
    fn truncation_corruption_and_trailing_bytes_are_rejected() {
        let frame = encode(42, b"uart\0\xff").unwrap();
        for end in 0..frame.len() {
            assert!(decode(&frame[..end]).is_err());
        }
        for offset in 0..frame.len() {
            let mut corrupt = frame.clone();
            corrupt[offset] ^= 1;
            assert!(decode(&corrupt).is_err());
        }
        let mut extra = frame;
        extra.push(0);
        assert!(decode(&extra).is_err());
    }
}
