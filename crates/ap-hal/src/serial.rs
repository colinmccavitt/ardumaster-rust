//! Serial ports, ported from `AP_HAL/UARTDriver.h`.
//!
//! Carries MAVLink to the ground station and NMEA/UBX from the GPS. Upstream's
//! `UARTDriver` is a large surface — flow control, DMA, parity, RTS/CTS pin
//! control — most of which is board configuration rather than flight logic.
//! Only the byte-stream core is ported here; the rest lands with a consumer
//! that needs it.
//!
//! # Absence is not a value
//!
//! Upstream `int16_t read()` returns **-1** when no byte is available, so the
//! return type spans both "a byte" and "no byte". A caller that stores it in a
//! `uint8_t` silently turns "empty" into `0xFF`. The port returns
//! `Option<u8>`, which cannot be misread that way.
//!
//! Partial writes are preserved: [`Serial::write`] returns how many bytes were
//! accepted, exactly as upstream's `size_t write()` does. A caller that assumes
//! the whole buffer went out is wrong on both.

use crate::Result;

/// A byte-oriented serial port. Upstream `AP_HAL::UARTDriver`.
pub trait Serial {
    /// Open the port at `baud`. Upstream `begin()`.
    fn begin(&mut self, baud: u32) -> Result<()>;

    /// Close the port. Upstream `end()`.
    fn end(&mut self) -> Result<()>;

    /// Whether the port has been opened. Upstream `is_initialized()`.
    fn is_initialized(&self) -> bool;

    /// Bytes available to read. Upstream `available()`.
    fn available(&self) -> usize;

    /// Free space in the transmit buffer, in bytes. Upstream `txspace()`.
    fn txspace(&self) -> usize;

    /// Read one byte, or `None` if the buffer is empty.
    ///
    /// Upstream returns `-1` for empty, encoding absence in the value.
    fn read_byte(&mut self) -> Option<u8>;

    /// Read into `buf`, returning how many bytes were read.
    ///
    /// May be fewer than `buf.len()`, including zero. Upstream `read(buf, n)`.
    fn read(&mut self, buf: &mut [u8]) -> usize;

    /// Write `buf`, returning how many bytes were accepted.
    ///
    /// May be fewer than `buf.len()` when the transmit buffer is full, matching
    /// upstream's `size_t write()`. Callers must handle the short write.
    fn write(&mut self, buf: &[u8]) -> usize;

    /// Whether bytes are still queued for transmission. Upstream `tx_pending()`.
    fn tx_pending(&self) -> bool;

    /// Discard buffered data. Upstream `flush()`.
    fn flush(&mut self) {}
}

/// An in-memory [`Serial`] backed by fixed buffers, for tests and SITL.
///
/// Sized by const generics so it stays stack-allocated with no allocator.
#[derive(Debug)]
pub struct LoopbackSerial<const N: usize> {
    rx: [u8; N],
    rx_len: usize,
    rx_pos: usize,
    tx: [u8; N],
    tx_len: usize,
    initialized: bool,
    baud: u32,
}

impl<const N: usize> Default for LoopbackSerial<N> {
    fn default() -> Self {
        Self {
            rx: [0; N],
            rx_len: 0,
            rx_pos: 0,
            tx: [0; N],
            tx_len: 0,
            initialized: false,
            baud: 0,
        }
    }
}

impl<const N: usize> LoopbackSerial<N> {
    /// A closed port with empty buffers.
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue bytes as if the peer had sent them.
    ///
    /// Returns how many were accepted, so a test can assert overflow behaviour
    /// rather than silently losing data.
    pub fn feed(&mut self, bytes: &[u8]) -> usize {
        let space = N - self.rx_len;
        let n = bytes.len().min(space);
        for (i, b) in bytes.iter().take(n).enumerate() {
            if let Some(slot) = self.rx.get_mut(self.rx_len + i) {
                *slot = *b;
            }
        }
        self.rx_len += n;
        n
    }

    /// Everything written to the port so far.
    pub fn written(&self) -> &[u8] {
        self.tx.get(..self.tx_len).unwrap_or(&[])
    }

    /// The baud rate the port was opened at.
    pub fn baud(&self) -> u32 {
        self.baud
    }
}

impl<const N: usize> Serial for LoopbackSerial<N> {
    fn begin(&mut self, baud: u32) -> Result<()> {
        self.baud = baud;
        self.initialized = true;
        Ok(())
    }

    fn end(&mut self) -> Result<()> {
        self.initialized = false;
        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }

    fn available(&self) -> usize {
        self.rx_len - self.rx_pos
    }

    fn txspace(&self) -> usize {
        N - self.tx_len
    }

    fn read_byte(&mut self) -> Option<u8> {
        if self.rx_pos >= self.rx_len {
            return None;
        }
        let b = self.rx.get(self.rx_pos).copied();
        self.rx_pos += 1;
        b
    }

    fn read(&mut self, buf: &mut [u8]) -> usize {
        let mut n = 0;
        for slot in buf.iter_mut() {
            match self.read_byte() {
                Some(b) => {
                    *slot = b;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }

    fn write(&mut self, buf: &[u8]) -> usize {
        let space = N - self.tx_len;
        let n = buf.len().min(space);
        for (i, b) in buf.iter().take(n).enumerate() {
            if let Some(slot) = self.tx.get_mut(self.tx_len + i) {
                *slot = *b;
            }
        }
        self.tx_len += n;
        n
    }

    fn tx_pending(&self) -> bool {
        false
    }

    fn flush(&mut self) {
        self.rx_pos = self.rx_len;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_bytes() {
        let mut s = LoopbackSerial::<16>::new();
        s.begin(57600).unwrap();
        assert!(s.is_initialized());
        assert_eq!(s.baud(), 57600);

        s.feed(b"HELLO");
        assert_eq!(s.available(), 5);

        let mut buf = [0u8; 5];
        assert_eq!(s.read(&mut buf), 5);
        assert_eq!(&buf, b"HELLO");
        assert_eq!(s.available(), 0);

        s.write(b"ACK");
        assert_eq!(s.written(), b"ACK");
    }

    /// Upstream read() returns -1 for empty, which a caller storing into a
    /// uint8_t silently turns into 0xFF. None cannot be misread that way.
    #[test]
    fn empty_read_is_none_not_a_sentinel() {
        let mut s = LoopbackSerial::<8>::new();
        assert_eq!(s.read_byte(), None);

        // a genuine 0xFF byte is Some(0xFF), distinct from empty
        s.feed(&[0xFF]);
        assert_eq!(s.read_byte(), Some(0xFF));
        assert_eq!(s.read_byte(), None);
    }

    /// Short writes are real and preserved: upstream size_t write() can accept
    /// fewer bytes than offered, and callers must handle it.
    #[test]
    fn write_reports_partial_acceptance() {
        let mut s = LoopbackSerial::<4>::new();
        let n = s.write(b"TOOLONG");
        assert_eq!(n, 4, "only txspace bytes are accepted");
        assert_eq!(s.written(), b"TOOL");
        assert_eq!(s.txspace(), 0);

        // a full buffer accepts nothing further rather than erroring
        assert_eq!(s.write(b"X"), 0);
    }

    /// A short read is likewise reported rather than padded.
    #[test]
    fn read_reports_partial_fill() {
        let mut s = LoopbackSerial::<8>::new();
        s.feed(b"AB");
        let mut buf = [0xEEu8; 4];
        assert_eq!(s.read(&mut buf), 2);
        assert_eq!(&buf[..2], b"AB");
        assert_eq!(buf[2], 0xEE, "untouched bytes are not overwritten");
    }

    #[test]
    fn feed_reports_overflow() {
        let mut s = LoopbackSerial::<4>::new();
        assert_eq!(s.feed(b"ABCDEF"), 4, "overflow is reported, not silent");
        assert_eq!(s.available(), 4);
    }

    #[test]
    fn flush_discards_pending_input() {
        let mut s = LoopbackSerial::<8>::new();
        s.feed(b"DATA");
        s.flush();
        assert_eq!(s.available(), 0);
        assert_eq!(s.read_byte(), None);
    }
}
