use core::sync::atomic::{AtomicU32, Ordering};

#[repr(transparent)]
pub struct Counter(AtomicU32);

impl Counter {
    const fn new() -> Self {
        Self(AtomicU32::new(0))
    }

    pub fn increment(&self) {
        let _ = self
            .0
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
                Some(value.saturating_add(1))
            });
    }

    pub fn add(&self, amount: usize) {
        let amount = u32::try_from(amount).unwrap_or(u32::MAX);
        let _ = self
            .0
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
                Some(value.saturating_add(amount))
            });
    }

    pub fn load(&self) -> u32 {
        self.0.load(Ordering::Relaxed)
    }
}

#[repr(C)]
pub struct FirmwareCounters {
    pub host_rx_errors: Counter,
    pub host_tx_errors: Counter,
    pub servo_rx_errors: Counter,
    pub servo_tx_errors: Counter,
    pub stale_servo_bytes: Counter,
    pub invalid_lengths: Counter,
    pub oversized_frames: Counter,
    pub incomplete_frame_timeouts: Counter,
    pub frames_forwarded: Counter,
    pub servo_bytes_forwarded: Counter,
}

impl FirmwareCounters {
    const fn new() -> Self {
        Self {
            host_rx_errors: Counter::new(),
            host_tx_errors: Counter::new(),
            servo_rx_errors: Counter::new(),
            servo_tx_errors: Counter::new(),
            stale_servo_bytes: Counter::new(),
            invalid_lengths: Counter::new(),
            oversized_frames: Counter::new(),
            incomplete_frame_timeouts: Counter::new(),
            frames_forwarded: Counter::new(),
            servo_bytes_forwarded: Counter::new(),
        }
    }
}

#[used]
#[unsafe(no_mangle)]
pub static FIRMWARE_COUNTERS: FirmwareCounters = FirmwareCounters::new();

#[cfg(test)]
mod tests {
    use super::Counter;

    #[test]
    fn counter_accumulates_values() {
        let counter = Counter::new();
        counter.increment();
        counter.add(41);
        assert_eq!(counter.load(), 42);
    }
}
