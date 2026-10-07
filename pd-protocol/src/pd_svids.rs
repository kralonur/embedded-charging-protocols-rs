//! Complete 16-bit SVID set. Zero is a terminator, not a supported SVID.
//! PD R3.2 V1.2 §§6.4.12.4, 8.6.2. No heap and no per-SVID mode array.
use crate::pd_cable_modes::{Status, response};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Progress {
    pub status: Status,
    pub count: u16,
    pub repeated: bool,
}

/// Interior-mutability boundary for caller-owned storage.
/// Calls are synchronous; no store lock may survive a PHY await.
pub trait Store {
    fn clear(&self);
    fn response(&self, objects: &[u32]) -> Option<Progress>;
    fn at(&self, index: u16) -> Option<u16>;
    fn contains(&self, svid: u16) -> bool;
}

pub struct Svids {
    bits: [u8; 8192],
    count: u16,
    last: [u32; 6],
    continuing: bool,
    started: bool,
}
impl Svids {
    pub const EMPTY: Self = Self {
        bits: [0; 8192],
        count: 0,
        last: [0; 6],
        continuing: false,
        started: false,
    };
    /// Reset in place: never create an 8 KiB temporary on an async stack.
    pub fn clear(&mut self) {
        self.bits.fill(0);
        self.count = 0;
        self.last.fill(0);
        self.continuing = false;
        self.started = false;
    }
    pub fn contains(&self, svid: u16) -> bool {
        svid != 0 && self.bits[svid as usize / 8] & (1 << (svid % 8)) != 0
    }
    pub fn count(&self) -> u16 {
        self.count
    }
    /// Ascending numeric order, explicitly not the responder's wire order.
    pub fn at(&self, index: u16) -> Option<u16> {
        if index >= self.count {
            return None;
        }
        let mut remaining = index as u32;
        for (byte_index, &byte) in self.bits.iter().enumerate() {
            let count = byte.count_ones();
            if remaining >= count {
                remaining -= count;
                continue;
            }
            for bit in 0..8 {
                if byte & (1 << bit) != 0 {
                    if remaining == 0 {
                        return Some((byte_index * 8 + bit) as u16);
                    }
                    remaining -= 1;
                }
            }
        }
        None
    }
    /// Validate completely before mutation, including replay/zero placement.
    pub fn response(&mut self, objects: &[u32]) -> Option<Progress> {
        if self.started && !self.continuing {
            return None;
        }
        let mut status = response(objects, 0xff00, 2)?;
        if status != Status::Ack {
            self.started = true;
            self.continuing = false;
            return Some(Progress {
                status,
                count: self.count,
                repeated: false,
            });
        }
        let mut values = [0u16; 12];
        let mut n = 0;
        let mut terminated = false;
        for (i, &word) in objects[1..].iter().enumerate() {
            for (half, value) in [(word >> 16) as u16, word as u16].into_iter().enumerate() {
                if value == 0 {
                    if i + 2 != objects.len() || half == 0 && word as u16 != 0 {
                        return None;
                    }
                    terminated = true;
                    break;
                }
                if values[..n].contains(&value) {
                    return None;
                }
                values[n] = value;
                n += 1;
            }
        }
        if !terminated && objects.len() != 7 || terminated && n == 0 && self.count == 0 {
            return None;
        }
        // §8.6.2.1: loss of GoodCRC can make a plug resend its previous list.
        let repeated = !terminated && self.continuing && objects[1..] == self.last;
        if !repeated && values[..n].iter().any(|&svid| self.contains(svid)) {
            return None;
        }
        if !repeated {
            for &svid in &values[..n] {
                self.bits[svid as usize / 8] |= 1 << (svid % 8);
                self.count = self.count.checked_add(1)?;
            }
        }
        self.started = true;
        self.continuing = !terminated;
        if !terminated {
            self.last.copy_from_slice(&objects[1..]);
            status = Status::Continuing;
        }
        Some(Progress {
            status,
            count: self.count,
            repeated,
        })
    }
}
impl Default for Svids {
    fn default() -> Self {
        Self::EMPTY
    }
}
impl Store for core::cell::RefCell<Svids> {
    fn clear(&self) {
        self.borrow_mut().clear();
    }
    fn response(&self, objects: &[u32]) -> Option<Progress> {
        self.borrow_mut().response(objects)
    }
    fn at(&self, index: u16) -> Option<u16> {
        self.borrow().at(index)
    }
    fn contains(&self, svid: u16) -> bool {
        self.borrow().contains(svid)
    }
}
