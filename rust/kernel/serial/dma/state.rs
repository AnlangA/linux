// SPDX-License-Identifier: GPL-2.0-only

//! Cancellation/completion bookkeeping shared by the driver and host tests.

#[derive(Clone, Copy, PartialEq)]
pub(super) enum Phase {
    Idle,
    Running,
    Stopping,
}

pub(super) struct State {
    pub(super) active: bool,
    pub(super) configuring: bool,
    pub(super) receive: bool,
    pub(super) tx: Phase,
    pub(super) rx: Phase,
    pub(super) tx_cookie: i32,
    pub(super) rx_cookie: i32,
    pub(super) tx_len: usize,
    pub(super) rx_cpu_safe: bool,
    pub(super) rx_failed: bool,
    pub(super) tx_failed: bool,
}

impl State {
    pub(super) fn complete_tx(&mut self) -> Option<usize> {
        if !self.active || self.tx != Phase::Running {
            return None;
        }
        let count = self.tx_len;
        self.tx_len = 0;
        self.tx = Phase::Idle;
        Some(count)
    }

    pub(super) fn cancel_tx(&mut self) -> bool {
        if !self.active || self.tx == Phase::Idle {
            return false;
        }
        self.tx_len = 0;
        self.tx = Phase::Stopping;
        true
    }

    pub(super) fn complete_rx(&mut self) -> bool {
        if !self.active || self.rx != Phase::Running {
            return false;
        }
        self.rx = Phase::Idle;
        self.rx_cpu_safe = true;
        true
    }
    pub(super) const fn new() -> Self {
        Self {
            active: false,
            configuring: false,
            receive: false,
            tx: Phase::Idle,
            rx: Phase::Idle,
            tx_cookie: 0,
            rx_cookie: 0,
            tx_len: 0,
            rx_cpu_safe: true,
            rx_failed: false,
            tx_failed: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transmit_completion_advances_exactly_once() {
        let mut state = State {
            active: true,
            tx: Phase::Running,
            tx_len: 256,
            ..State::new()
        };
        assert_eq!(state.complete_tx(), Some(256));
        assert_eq!(state.complete_tx(), None);
    }

    #[test]
    fn flush_does_not_advance_the_reset_tty_queue() {
        let mut state = State {
            active: true,
            tx: Phase::Running,
            tx_len: 256,
            ..State::new()
        };
        assert!(state.cancel_tx());
        assert!(state.tx == Phase::Stopping);
        assert_eq!(state.complete_tx(), None);
        assert_eq!(state.tx_len, 0);
    }

    #[test]
    fn late_completion_after_close_is_ignored() {
        let mut state = State {
            tx: Phase::Running,
            rx: Phase::Running,
            tx_len: 17,
            ..State::new()
        };
        assert_eq!(state.complete_tx(), None);
        assert!(!state.complete_rx());
    }

    #[test]
    fn timeout_and_full_receive_callback_cannot_both_deliver() {
        let mut state = State {
            active: true,
            rx: Phase::Stopping,
            ..State::new()
        };
        assert!(!state.complete_rx());
        state.rx = Phase::Running;
        assert!(state.complete_rx());
        assert!(!state.complete_rx());
        assert!(state.rx_cpu_safe);
    }
}
