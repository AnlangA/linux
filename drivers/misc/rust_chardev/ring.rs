// SPDX-License-Identifier: GPL-2.0-only

//! Storage-independent bounded ring indices. No allocation or unsafe code.

use core::ops::Range;

pub(crate) struct Ring {
    capacity: usize,
    head: usize,
    len: usize,
}

impl Ring {
    pub(crate) const fn new(capacity: usize) -> Self {
        assert!(capacity > 0);
        Self {
            capacity,
            head: 0,
            len: 0,
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.len == 0
    }
    pub(crate) fn is_full(&self) -> bool {
        self.len == self.capacity
    }

    pub(crate) fn readable(&self, limit: usize) -> Range<usize> {
        self.head..self.head + self.len.min(self.capacity - self.head).min(limit)
    }

    pub(crate) fn writable(&self, limit: usize) -> Range<usize> {
        // Avoid overflowing head + len even if used with very large storage.
        let tail = if self.len >= self.capacity - self.head {
            self.len - (self.capacity - self.head)
        } else {
            self.head + self.len
        };
        tail..tail
            + (self.capacity - self.len)
                .min(self.capacity - tail)
                .min(limit)
    }

    pub(crate) fn produce(&mut self, count: usize) {
        assert!(count <= self.writable(usize::MAX).len());
        self.len += count;
    }

    pub(crate) fn consume(&mut self, count: usize) {
        assert!(count <= self.readable(usize::MAX).len());
        self.head = (self.head + count) % self.capacity;
        self.len -= count;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn wraparound_and_partial_copies_match_reference_queue() {
        for capacity in [1, 2, 7, 64, 4096] {
            let mut ring = Ring::new(capacity);
            let mut data = vec![0u8; capacity];
            let mut reference = VecDeque::new();
            let mut seed = 7u32;
            for _ in 0..20_000 {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                let limit = (seed as usize >> 8) % 100;
                if seed & 1 == 0 {
                    let range = ring.writable(limit);
                    // Simulate short copy_from_user(), including a zero-byte fault.
                    let count = if range.is_empty() {
                        0
                    } else {
                        (seed as usize >> 16) % (range.len() + 1)
                    };
                    for (i, byte) in data.iter_mut().enumerate().skip(range.start).take(count) {
                        *byte = seed.wrapping_add(i as u32) as u8;
                        reference.push_back(*byte);
                    }
                    ring.produce(count);
                } else {
                    let range = ring.readable(limit);
                    for byte in &data[range.clone()] {
                        assert_eq!(reference.pop_front(), Some(*byte));
                    }
                    ring.consume(range.len());
                }
                assert_eq!(ring.is_empty(), reference.is_empty());
                assert_eq!(ring.is_full(), reference.len() == capacity);
            }
        }
    }

    #[test]
    fn full_empty_and_zero_length_operations() {
        let mut ring = Ring::new(4);
        assert!(ring.is_empty());
        assert_eq!(ring.writable(0), 0..0);
        ring.produce(4);
        assert!(ring.is_full());
        assert!(ring.writable(1).is_empty());
        ring.consume(3);
        assert_eq!(ring.writable(4), 0..3);
        ring.produce(3);
        assert_eq!(ring.readable(4), 3..4);
        ring.consume(1);
        assert_eq!(ring.readable(4), 0..3);
    }
}
