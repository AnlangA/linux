// SPDX-License-Identifier: GPL-2.0-only

//! Pure register encoding, also exercised by the userspace regression suite.

pub(crate) struct Divisor {
    pub integer: u16,
    pub fraction: u32,
    pub actual_baud: u32,
}

/// Computes a rounded fixed-point divisor, carrying a rounded fraction into
/// the integer part. Rejects rates with more than two percent quantization error.
pub(crate) fn divisor(clock: u32, baud: u32, fraction_bits: u8) -> Option<Divisor> {
    if clock == 0 || baud == 0 || fraction_bits > 16 {
        return None;
    }
    let scale = 1u64 << fraction_bits;
    let numerator = u64::from(clock) * scale;
    let denominator = 16 * u64::from(baud);
    let fixed = (numerator + denominator / 2) / denominator;
    if fixed < scale || fixed >= 65536 * scale {
        return None;
    }
    let actual = ((numerator + 8 * fixed) / (16 * fixed)) as u32;
    if u64::from(actual.abs_diff(baud)) * 100 > u64::from(baud) * 2 {
        return None;
    }
    Some(Divisor {
        integer: (fixed / scale) as u16,
        fraction: (fixed % scale) as u32,
        actual_baud: actual,
    })
}

/// Chooses the input clock for `baud`: the fixed reference whenever it divides
/// well enough, otherwise sixteen times the baud rate for the CRU to synthesize.
pub(crate) fn baud_clock(reference: u32, baud: u32, fraction_bits: u8) -> u32 {
    if divisor(reference, baud, fraction_bits).is_some() {
        reference
    } else {
        baud.saturating_mul(16)
    }
}

pub(crate) fn line_control(bits: u8, two_stops: bool, parity: bool, odd: bool) -> u32 {
    u32::from(bits.saturating_sub(5).min(3))
        | (u32::from(two_stops) << 2)
        | (u32::from(parity) << 3)
        | (u32::from(parity && !odd) << 4)
}

pub(crate) fn receive_status(lsr: u32) -> u32 {
    // serial::receive uses normalized flags: overrun, parity, frame, break.
    (lsr >> 1) & 0x0f
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fractional_rates_and_rounding_carry() {
        for rate in [
            9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1500000,
        ] {
            let d = divisor(24_000_000, rate, 6).unwrap();
            assert!(d.integer > 0 && d.fraction < 64);
            assert!(u64::from(d.actual_baud.abs_diff(rate)) * 100 <= u64::from(rate) * 2);
        }
        let d = divisor(24_000_000, 750_001, 6).unwrap();
        assert_eq!((d.integer, d.fraction), (2, 0));
        assert!(divisor(24_000_000, 921600, 0).is_none());
        assert!(divisor(0, 115200, 6).is_none());
        assert!(divisor(24_000_000, 0, 6).is_none());
        assert!(divisor(u32::MAX, 1, 16).is_none());
        assert!(divisor(24_000_000, 115200, 32).is_none());
    }

    #[test]
    fn reference_clock_is_kept_when_it_divides_well() {
        for rate in [9600, 57600, 115200, 1_500_000] {
            assert_eq!(baud_clock(24_000_000, rate, 0), 24_000_000);
        }
        assert_eq!(baud_clock(24_000_000, 230_400, 0), 3_686_400);
        assert_eq!(baud_clock(24_000_000, 921_600, 0), 14_745_600);
        assert_eq!(baud_clock(24_000_000, 921_600, 6), 24_000_000);
        assert_eq!(baud_clock(24_000_000, u32::MAX, 0), u32::MAX);
    }

    #[test]
    fn supported_framing() {
        assert_eq!(line_control(8, false, false, false), 0x03);
        assert_eq!(line_control(7, false, true, false), 0x1a);
        assert_eq!(line_control(8, true, true, true), 0x0f);
        assert_eq!(line_control(5, false, false, false), 0x00);
    }

    #[test]
    fn status_excludes_data_and_transmit_bits() {
        assert_eq!(receive_status(0x61), 0);
        assert_eq!(receive_status(0x1e), 0x0f);
        assert_eq!(receive_status(0x04), 2);
    }
}
