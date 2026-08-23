/// Byte repeated twice at the beginning of every servo frame.
pub const HEADER: u8 = 0x55;

/// Smallest valid value of the protocol `Length` field.
pub const MIN_LENGTH: u8 = 3;

/// A complete frame stored without allocation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Frame<const CAPACITY: usize> {
    bytes: [u8; CAPACITY],
    len: usize,
}

impl<const CAPACITY: usize> Frame<CAPACITY> {
    /// Exact bytes received from the transport.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    /// Physical frame size in bytes.
    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// Result of feeding one byte to [`FrameDecoder`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PushResult<const CAPACITY: usize> {
    /// No complete frame or framing error yet.
    Pending,
    /// One complete, byte-for-byte unchanged frame.
    Frame(Frame<CAPACITY>),
    /// `Length` was smaller than the protocol minimum.
    InvalidLength(u8),
    /// The announced physical size does not fit the bounded buffer.
    FrameTooLarge { announced: usize, capacity: usize },
}

/// Incremental, allocation-free frame-boundary decoder.
///
/// Noise before `55 55` is ignored. Repeated `55` bytes are reused as a
/// possible new header, allowing recovery without an external reset.
pub struct FrameDecoder<const CAPACITY: usize> {
    bytes: [u8; CAPACITY],
    received: usize,
    expected: usize,
    header_bytes: u8,
}

impl<const CAPACITY: usize> Default for FrameDecoder<CAPACITY> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAPACITY: usize> FrameDecoder<CAPACITY> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; CAPACITY],
            received: 0,
            expected: 0,
            header_bytes: 0,
        }
    }

    /// Drops an incomplete frame and returns to header search.
    pub fn reset(&mut self) {
        self.received = 0;
        self.expected = 0;
        self.header_bytes = 0;
    }

    /// Whether a possible header or frame is currently incomplete.
    pub const fn is_collecting(&self) -> bool {
        self.received != 0 || self.header_bytes != 0
    }

    pub fn push(&mut self, byte: u8) -> PushResult<CAPACITY> {
        if self.received == 0 {
            return self.push_header(byte);
        }

        // A tiny capacity can find a header but cannot inspect Length safely.
        if self.received >= CAPACITY {
            self.reset();
            return PushResult::FrameTooLarge {
                announced: self.expected.max(self.received + 1),
                capacity: CAPACITY,
            };
        }

        self.bytes[self.received] = byte;
        self.received += 1;

        if self.received == 4 {
            if byte < MIN_LENGTH {
                self.recover_header_from(byte);
                return PushResult::InvalidLength(byte);
            }

            self.expected = usize::from(byte) + 3;
            if self.expected > CAPACITY {
                let announced = self.expected;
                self.recover_header_from(byte);
                return PushResult::FrameTooLarge {
                    announced,
                    capacity: CAPACITY,
                };
            }
        }

        if self.expected != 0 && self.received == self.expected {
            let frame = Frame {
                bytes: self.bytes,
                len: self.received,
            };
            self.reset();
            PushResult::Frame(frame)
        } else {
            PushResult::Pending
        }
    }

    fn push_header(&mut self, byte: u8) -> PushResult<CAPACITY> {
        if byte != HEADER {
            self.header_bytes = 0;
            return PushResult::Pending;
        }

        self.header_bytes += 1;
        if self.header_bytes < 2 {
            return PushResult::Pending;
        }

        if CAPACITY < 2 {
            self.header_bytes = 1;
            return PushResult::FrameTooLarge {
                announced: 2,
                capacity: CAPACITY,
            };
        }

        self.bytes[0] = HEADER;
        self.bytes[1] = HEADER;
        self.received = 2;
        self.header_bytes = 0;
        PushResult::Pending
    }

    fn recover_header_from(&mut self, last_byte: u8) {
        self.expected = 0;
        if self.bytes[2] == HEADER {
            self.bytes[0] = HEADER;
            self.bytes[1] = HEADER;
            self.bytes[2] = last_byte;
            self.received = 3;
            self.header_bytes = 0;
        } else {
            self.received = 0;
            self.header_bytes = u8::from(last_byte == HEADER);
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::{FrameDecoder, PushResult};
    use std::vec::Vec;

    const MOVE: &[u8] = &[0x55, 0x55, 0x01, 0x07, 0x01, 0xf4, 0x01, 0xe8, 0x03, 0x16];
    const READ: &[u8] = &[0x55, 0x55, 0x01, 0x03, 0x1c, 0xdf];

    fn collect<const N: usize>(decoder: &mut FrameDecoder<N>, input: &[u8]) -> Vec<Vec<u8>> {
        input
            .iter()
            .filter_map(|&byte| match decoder.push(byte) {
                PushResult::Frame(frame) => Some(frame.as_bytes().to_vec()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn accepts_a_frame_split_across_reads() {
        let mut decoder = FrameDecoder::<16>::new();
        assert!(collect(&mut decoder, &MOVE[..4]).is_empty());
        assert_eq!(collect(&mut decoder, &MOVE[4..]), [MOVE]);
    }

    #[test]
    fn emits_multiple_frames_from_one_read() {
        let mut decoder = FrameDecoder::<16>::new();
        let input: Vec<_> = MOVE.iter().chain(READ).copied().collect();
        assert_eq!(collect(&mut decoder, &input), [MOVE, READ]);
    }

    #[test]
    fn skips_noise_and_reuses_repeated_header_bytes() {
        let mut decoder = FrameDecoder::<16>::new();
        let input: Vec<_> = [0x00, 0xaa, 0x55].iter().chain(READ).copied().collect();
        assert_eq!(collect(&mut decoder, &input), [READ]);
    }

    #[test]
    fn reports_invalid_length_then_recovers() {
        let mut decoder = FrameDecoder::<16>::new();
        let input: Vec<_> = [0x55, 0x55, 0x01, 0x02]
            .iter()
            .chain(READ)
            .copied()
            .collect();
        let mut invalid = false;
        let mut frames = Vec::new();
        for byte in input {
            match decoder.push(byte) {
                PushResult::InvalidLength(2) => invalid = true,
                PushResult::Frame(frame) => frames.push(frame.as_bytes().to_vec()),
                _ => {}
            }
        }
        assert!(invalid);
        assert_eq!(frames, [READ]);
    }

    #[test]
    fn reports_bounded_buffer_overflow_then_recovers() {
        let mut decoder = FrameDecoder::<8>::new();
        let input: Vec<_> = MOVE.iter().chain(READ).copied().collect();
        let mut too_large = false;
        let mut frames = Vec::new();
        for byte in input {
            match decoder.push(byte) {
                PushResult::FrameTooLarge {
                    announced: 10,
                    capacity: 8,
                } => too_large = true,
                PushResult::Frame(frame) => frames.push(frame.as_bytes().to_vec()),
                _ => {}
            }
        }
        assert!(too_large);
        assert_eq!(frames, [READ]);
    }

    #[test]
    fn reset_discards_an_incomplete_frame() {
        let mut decoder = FrameDecoder::<16>::new();
        assert!(collect(&mut decoder, &MOVE[..5]).is_empty());
        decoder.reset();
        assert_eq!(collect(&mut decoder, READ), [READ]);
    }

    #[test]
    fn reports_whether_an_incomplete_frame_is_being_collected() {
        let mut decoder = FrameDecoder::<16>::new();
        assert!(!decoder.is_collecting());
        decoder.push(0x55);
        assert!(decoder.is_collecting());
        decoder.reset();
        assert!(!decoder.is_collecting());
        assert_eq!(collect(&mut decoder, READ), [READ]);
        assert!(!decoder.is_collecting());
    }

    #[test]
    fn sustains_ten_thousand_back_to_back_frames() {
        let mut decoder = FrameDecoder::<16>::new();
        let mut completed = 0_u32;

        for _ in 0..10_000 {
            for &byte in READ {
                if let PushResult::Frame(frame) = decoder.push(byte) {
                    assert_eq!(frame.as_bytes(), READ);
                    completed += 1;
                }
            }
        }

        assert_eq!(completed, 10_000);
        assert!(!decoder.is_collecting());
    }

    #[test]
    fn recovers_after_dense_noise_and_malformed_frames() {
        let mut decoder = FrameDecoder::<16>::new();
        let malformed = [
            0xaa, 0x55, 0x00, 0x55, 0x55, 0x01, 0x00, 0xff, 0x55, 0x55, 0x01, 0x02,
        ];

        assert!(collect(&mut decoder, &malformed).is_empty());
        decoder.reset(); // Models the 50 ms transport-level inactivity timeout.
        assert_eq!(collect(&mut decoder, READ), [READ]);
    }
}
