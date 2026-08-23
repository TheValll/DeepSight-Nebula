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
use esp_hal::time::{Duration, Instant};
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_println as _;
use rust_firmware::diagnostics::FIRMWARE_COUNTERS;
use rust_firmware::framing::{FrameDecoder, PushResult};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const HOST_BAUDRATE: u32 = 115_200;
const SERVO_BAUDRATE: u32 = 115_200;
const HOST_FRAME_CAPACITY: usize = 258;
const HOST_FRAME_TIMEOUT: Duration = Duration::from_millis(50);
const UART_CHUNK_SIZE: usize = 64;

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

fn discard_stale_servo_bytes(servo_uart: &mut Uart<'_, Blocking>) {
    let mut discard = [0_u8; UART_CHUNK_SIZE];

    while servo_uart.read_ready() {
        match servo_uart.read(&mut discard) {
            Ok(count) => {
                FIRMWARE_COUNTERS.stale_servo_bytes.add(count);
            }
            Err(_) => {
                FIRMWARE_COUNTERS.servo_rx_errors.increment();
                break;
            }
        }
    }
}

fn forward_servo_bytes(servo_uart: &mut Uart<'_, Blocking>, host_uart: &mut Uart<'_, Blocking>) {
    let mut response = [0_u8; UART_CHUNK_SIZE];

    while servo_uart.read_ready() {
        match servo_uart.read(&mut response) {
            Ok(count) => {
                if write_all(host_uart, &response[..count]).is_err() {
                    FIRMWARE_COUNTERS.host_tx_errors.increment();
                    return;
                }
                FIRMWARE_COUNTERS.servo_bytes_forwarded.add(count);
            }
            Err(_) => {
                FIRMWARE_COUNTERS.servo_rx_errors.increment();
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
    let mut last_host_activity = Instant::now();

    loop {
        if decoder.is_collecting() && last_host_activity.elapsed() >= HOST_FRAME_TIMEOUT {
            decoder.reset();
            direction.receive();
            FIRMWARE_COUNTERS.incomplete_frame_timeouts.increment();
        }

        forward_servo_bytes(&mut servo_uart, &mut host_uart);

        if !host_uart.read_ready() {
            core::hint::spin_loop();
            continue;
        }

        let received = match host_uart.read(&mut host_bytes) {
            Ok(received) => received,
            Err(_) => {
                FIRMWARE_COUNTERS.host_rx_errors.increment();
                decoder.reset();
                direction.receive();
                continue;
            }
        };

        for &byte in &host_bytes[..received] {
            last_host_activity = Instant::now();
            match decoder.push(byte) {
                PushResult::Pending => {}
                PushResult::InvalidLength(_) => {
                    FIRMWARE_COUNTERS.invalid_lengths.increment();
                }
                PushResult::FrameTooLarge { .. } => {
                    FIRMWARE_COUNTERS.oversized_frames.increment();
                }
                PushResult::Frame(frame) => {
                    discard_stale_servo_bytes(&mut servo_uart);

                    direction.transmit();
                    let transmission = write_all(&mut servo_uart, frame.as_bytes())
                        .and_then(|()| servo_uart.flush().map_err(|_| ()));
                    // This must run on both success and failure.
                    direction.receive();

                    if transmission.is_err() {
                        FIRMWARE_COUNTERS.servo_tx_errors.increment();
                    } else {
                        FIRMWARE_COUNTERS.frames_forwarded.increment();
                    }
                }
            }
        }
    }
}
