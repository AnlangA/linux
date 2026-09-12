// SPDX-License-Identifier: GPL-2.0-only

use rs485_test::*;
use std::{
    fs::File,
    io,
    os::fd::{AsRawFd, FromRawFd},
    time::{Duration, Instant},
};

fn pair() -> io::Result<(File, File)> {
    let (mut master, mut slave) = (-1, -1);
    // SAFETY: Writable output descriptors are supplied; null optional parameters
    // ask libc for default PTY settings and no name buffer.
    if unsafe {
        libc::openpty(
            &mut master,
            &mut slave,
            std::ptr::null_mut(),
            std::ptr::null(),
            std::ptr::null(),
        )
    } < 0
    {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: openpty returned distinct, owned descriptors on success.
    let files = unsafe { (File::from_raw_fd(master), File::from_raw_fd(slave)) };
    for file in [&files.0, &files.1] {
        // SAFETY: The file is live, and the zeroed termios is writable storage.
        let mut termios: libc::termios = unsafe { std::mem::zeroed() };
        // SAFETY: tcgetattr initializes the provided termios for a live PTY.
        if unsafe { libc::tcgetattr(file.as_raw_fd(), &mut termios) } < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: termios is initialized and exclusively borrowed.
        unsafe {
            libc::cfmakeraw(&mut termios);
        }
        // SAFETY: All arguments refer to live descriptors/initialized termios.
        if unsafe { libc::tcsetattr(file.as_raw_fd(), libc::TCSANOW, &termios) } < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: fcntl operates on this live descriptor with a valid status flag.
        if unsafe { libc::fcntl(file.as_raw_fd(), libc::F_SETFL, libc::O_NONBLOCK) } < 0 {
            return Err(io::Error::last_os_error());
        }
    }
    Ok(files)
}

#[test]
fn fragmented_frames_and_echo_over_a_real_pty() {
    let (mut host, mut peer) = pair().unwrap();
    let worker = std::thread::spawn(move || {
        for sequence in 0..32 {
            let frame = read_frame(&mut peer, deadline(Duration::from_secs(3)).unwrap()).unwrap();
            assert_eq!(decode(&frame).unwrap().0, sequence);
            // Exercise headers/CRC split across arbitrary reads, not just payload.
            for chunk in frame.chunks(3) {
                write_deadline(&mut peer, chunk, deadline(Duration::from_secs(3)).unwrap())
                    .unwrap();
            }
        }
        peer
    });
    for sequence in 0..32 {
        let frame = encode(sequence, &vec![sequence as u8; sequence as usize * 31]).unwrap();
        write_deadline(&mut host, &frame, deadline(Duration::from_secs(3)).unwrap()).unwrap();
        assert_eq!(
            read_frame(&mut host, deadline(Duration::from_secs(3)).unwrap()).unwrap(),
            frame
        );
    }
    drop(worker.join().unwrap());
}

#[test]
fn no_response_expires_and_closed_peer_is_reported() {
    let (mut host, peer) = pair().unwrap();
    let start = Instant::now();
    let err = read_frame(&mut host, deadline(Duration::from_millis(30)).unwrap()).unwrap_err();
    assert_eq!(err.kind(), io::ErrorKind::TimedOut);
    assert!(start.elapsed() < Duration::from_secs(1));
    drop(peer);
    assert!(read_frame(&mut host, deadline(Duration::from_secs(1)).unwrap()).is_err());
}
