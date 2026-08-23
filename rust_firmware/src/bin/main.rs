#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_backtrace as _;
use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::main;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_println as _;
use rust_firmware::framing::{FrameDecoder, PushResult};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const HOST_BAUDRATE: u32 = 115_200;
const SERVO_BAUDRATE: u32 = 115_200;
const HOST_FRAME_CAPACITY: usize = 258;
const UART_CHUNK_SIZE: usize = 64;

#[derive(Default)]
struct Counters {
    host_rx_errors: u32,
    host_tx_errors: u32,
    servo_rx_errors: u32,
    servo_tx_errors: u32,
    stale_servo_bytes: u32,
    invalid_lengths: u32,
    oversized_frames: u32,
}

/// Owns the two half-duplex direction signals.
///
/// The active-high polarity comes from the signal names in the current
/// specification and must be checked on the physical controller before use
/// with connected servos.
struct BusDirection<'d> {
    tx_enable: Output<'d>,
    rx_enable: Output<'d>,
}

impl<'d> BusDirection<'d> {
    fn new(
        tx_enable: esp_hal::peripherals::GPIO25<'d>,
        rx_enable: esp_hal::peripherals::GPIO12<'d>,
    ) -> Self {
        let output_config = OutputConfig::default();

        // Configure TX inactive first. GPIO12 is only driven after reset
        // strapping has been sampled by the chip.
        let tx_enable = Output::new(tx_enable, Level::Low, output_config);
        let rx_enable = Output::new(rx_enable, Level::High, output_config);

        Self {
            tx_enable,
            rx_enable,
        }
    }

    fn receive(&mut self) {
        self.tx_enable.set_low();
        self.rx_enable.set_high();
    }

    fn transmit(&mut self) {
        self.rx_enable.set_low();
        self.tx_enable.set_high();
    }
}

fn write_all(uart: &mut Uart<'_, Blocking>, mut bytes: &[u8]) -> Result<(), ()> {
    while !bytes.is_empty() {
        match uart.write(bytes) {
            Ok(written) if written != 0 => bytes = &bytes[written..],
            Ok(_) | Err(_) => return Err(()),
        }
    }
    Ok(())
}

fn discard_stale_servo_bytes(servo_uart: &mut Uart<'_, Blocking>, counters: &mut Counters) {
    let mut discard = [0_u8; UART_CHUNK_SIZE];

    while servo_uart.read_ready() {
        match servo_uart.read(&mut discard) {
            Ok(count) => {
                counters.stale_servo_bytes =
                    counters.stale_servo_bytes.saturating_add(count as u32);
            }
            Err(_) => {
                counters.servo_rx_errors = counters.servo_rx_errors.saturating_add(1);
                break;
            }
        }
    }
}

fn forward_servo_bytes(
    servo_uart: &mut Uart<'_, Blocking>,
    host_uart: &mut Uart<'_, Blocking>,
    counters: &mut Counters,
) {
    let mut response = [0_u8; UART_CHUNK_SIZE];

    while servo_uart.read_ready() {
        match servo_uart.read(&mut response) {
            Ok(count) => {
                if write_all(host_uart, &response[..count]).is_err() {
                    counters.host_tx_errors = counters.host_tx_errors.saturating_add(1);
                    return;
                }
            }
            Err(_) => {
                counters.servo_rx_errors = counters.servo_rx_errors.saturating_add(1);
                return;
            }
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32 -o unstable-hal -o esp-backtrace -o defmt -o esp32-wroom-32d -o wokwi -o vscode

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let host_config = UartConfig::default().with_baudrate(HOST_BAUDRATE);
    let mut host_uart = Uart::new(peripherals.UART0, host_config)
        .expect("valid host UART configuration")
        .with_rx(peripherals.GPIO3)
        .with_tx(peripherals.GPIO1);

    let servo_config = UartConfig::default().with_baudrate(SERVO_BAUDRATE);
    let mut servo_uart = Uart::new(peripherals.UART2, servo_config)
        .expect("valid servo UART configuration")
        .with_rx(peripherals.GPIO35)
        .with_tx(peripherals.GPIO26);

    let mut direction = BusDirection::new(peripherals.GPIO25, peripherals.GPIO12);
    direction.receive();

    let mut decoder = FrameDecoder::<HOST_FRAME_CAPACITY>::new();
    let mut host_bytes = [0_u8; UART_CHUNK_SIZE];
    let mut counters = Counters::default();

    loop {
        // Servo traffic has priority so a response is moved out of the UART2
        // FIFO with minimal latency.
        forward_servo_bytes(&mut servo_uart, &mut host_uart, &mut counters);

        if !host_uart.read_ready() {
            core::hint::spin_loop();
            continue;
        }

        let received = match host_uart.read(&mut host_bytes) {
            Ok(received) => received,
            Err(_) => {
                counters.host_rx_errors = counters.host_rx_errors.saturating_add(1);
                decoder.reset();
                continue;
            }
        };

        for &byte in &host_bytes[..received] {
            match decoder.push(byte) {
                PushResult::Pending => {}
                PushResult::InvalidLength(_) => {
                    counters.invalid_lengths = counters.invalid_lengths.saturating_add(1);
                }
                PushResult::FrameTooLarge { .. } => {
                    counters.oversized_frames = counters.oversized_frames.saturating_add(1);
                }
                PushResult::Frame(frame) => {
                    discard_stale_servo_bytes(&mut servo_uart, &mut counters);

                    direction.transmit();
                    let transmission = write_all(&mut servo_uart, frame.as_bytes())
                        .and_then(|()| servo_uart.flush().map_err(|_| ()));
                    // This must run on both success and failure.
                    direction.receive();

                    if transmission.is_err() {
                        counters.servo_tx_errors = counters.servo_tx_errors.saturating_add(1);
                    }
                }
            }
        }
    }
}
