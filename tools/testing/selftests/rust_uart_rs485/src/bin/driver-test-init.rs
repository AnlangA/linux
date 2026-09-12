// SPDX-License-Identifier: GPL-2.0-only

//! Disposable QEMU init process. Never run this on a normal host.

use std::{
    fs::{self, File},
    io,
    os::fd::AsRawFd,
    process::Command,
};

fn load(path: &str) -> io::Result<()> {
    let file = File::open(path)?;
    // SAFETY: The file is a readable module and parameters is a live empty C string.
    let result =
        unsafe { libc::syscall(libc::SYS_finit_module, file.as_raw_fd(), c"".as_ptr(), 0) };
    if result < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

fn unload(name: &std::ffi::CStr) -> io::Result<()> {
    // SAFETY: The module name is a valid C string; no force-unload flag is used.
    let result = unsafe { libc::syscall(libc::SYS_delete_module, name.as_ptr(), libc::O_NONBLOCK) };
    if result < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

fn test() -> io::Result<()> {
    fs::create_dir_all("/dev")?;
    fs::create_dir_all("/proc")?;
    // SAFETY: All mount strings are valid and this runs only as disposable PID 1.
    if unsafe {
        libc::mount(
            c"devtmpfs".as_ptr(),
            c"/dev".as_ptr(),
            c"devtmpfs".as_ptr(),
            0,
            std::ptr::null(),
        )
    } < 0
    {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: Same mount preconditions as above.
    if unsafe {
        libc::mount(
            c"proc".as_ptr(),
            c"/proc".as_ptr(),
            c"proc".as_ptr(),
            0,
            std::ptr::null(),
        )
    } < 0
    {
        return Err(io::Error::last_os_error());
    }
    for cycle in 0..3 {
        load("/rust_chardev.ko")?;
        let file = File::open("/dev/rust-chardev")?;
        match unload(c"rust_chardev") {
            Err(error) if error.raw_os_error() == Some(libc::EWOULDBLOCK) => {}
            other => return Err(io::Error::other(format!("unload while open: {other:?}"))),
        }
        drop(file);
        let status = Command::new("/rs485-test").arg("chardev").status()?;
        if !status.success() {
            return Err(io::Error::other("character-device regression failed"));
        }
        unload(c"rust_chardev")?;
        println!("QEMU cycle {cycle}: module lifecycle PASS");
    }
    load("/rust_dw_uart.ko")?;
    unload(c"rust_dw_uart")?;
    println!("QEMU: UART module registration/unregistration PASS (no RK3588 hardware)");
    Ok(())
}

fn main() {
    // SAFETY: getpid has no preconditions.
    if unsafe { libc::getpid() } != 1 {
        eprintln!("This test init must run as QEMU PID 1.");
        std::process::exit(2);
    }
    match test() {
        Ok(()) => println!("RUST_DRIVER_QEMU_RESULT=PASS"),
        Err(error) => println!("RUST_DRIVER_QEMU_RESULT=FAIL: {error}"),
    }
    // SAFETY: Power off only this disposable guest, after reporting its result.
    unsafe {
        libc::sync();
        libc::reboot(libc::RB_POWER_OFF);
    }
    loop {
        std::thread::park();
    }
}
