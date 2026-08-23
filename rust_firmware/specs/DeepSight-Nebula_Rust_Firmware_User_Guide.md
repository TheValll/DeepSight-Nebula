# DeepSight-Nebula — Rust Firmware User Guide

**Status:** Operational
**Version:** 1.0
**Date:** 2026-08-23

---

## 1. Purpose

This guide explains how to build, flash and operate the DeepSight-Nebula Rust
firmware on the Hiwonder xArm ESP32 controller.

The firmware is a binary transport bridge:

```text
Host PC
   │
   │ USB serial / CH340 / UART0
   │ 115200 baud, binary frames
   ▼
ESP32-WROOM-32D
   │
   │ UART2, half-duplex
   ▼
Servo bus
```

The firmware does not accept MicroPython expressions, text commands, JSON or
ROS 2 messages. The host must send complete binary servo frames.

---

## 2. Supported Hardware

The validated controller has the following characteristics:

| Component | Value |
| --------- | ----- |
| MCU | ESP32 revision 1.1 |
| Module | ESP32-WROOM-32D |
| Crystal | 40 MHz |
| Flash | 4 MiB |
| USB-to-serial converter | QinHeng CH340 (`1a86:7523`) |
| Linux serial device | Usually `/dev/ttyUSB0` |
| Stable Linux path | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |

### 2.1 Servo bus pins

| Signal | GPIO | Active level |
| ------ | ---: | -----------: |
| UART2 TX | 26 | UART |
| UART2 RX | 35 | UART |
| `tx_en` | 25 | High |
| `rx_en` | 12 | High |

Direction levels are:

| Mode | GPIO25 `tx_en` | GPIO12 `rx_en` |
| ---- | -------------: | -------------: |
| RX | 0 | 1 |
| TX | 1 | 0 |

GPIO12 is an ESP32 strapping pin. This assignment is specific to the original
xArm controller and was recovered from its stock MicroPython firmware. Do not
reuse this firmware on another board without checking its schematic.

---

## 3. Safety

### 3.1 Before flashing

1. Place the arm in a mechanically stable position.
2. Keep people and objects outside the arm workspace.
3. Turn off the main servo power supply.
4. Leave only the ESP32 USB cable connected.
5. Verify that the selected serial port belongs to the CH340 controller.

The ESP32 may be powered from USB during flashing. The servos must remain
unpowered until flashing has completed and the serial port has reappeared.

### 3.2 During operation

- Keep the main power switch accessible.
- Test read-only commands before motion commands.
- Use slow motion durations for the first test of a new host program.
- Never send textual commands from the old MicroPython driver.
- Keep at most one request expecting a response in flight.

If host communication is lost, the firmware does not generate another command
and returns to RX. It does not inject an emergency-stop command. A movement
already accepted by a servo continues for its encoded duration.

---

## 4. Host Setup

### 4.1 Identify the serial converter

```bash
lsusb
```

The validated converter appears as:

```text
1a86:7523 QinHeng Electronics CH340 serial converter
```

Find the associated port:

```bash
find /dev -maxdepth 1 \( -name 'ttyUSB*' -o -name 'ttyACM*' \) -ls
```

### 4.2 Serial permissions

Add the current user to the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in for the group change to take effect. For temporary access
without closing the current session:

```bash
sudo setfacl -m u:"$USER":rw /dev/ttyUSB0
```

Verify access:

```bash
test -r /dev/ttyUSB0 -a -w /dev/ttyUSB0 && echo "Serial access OK"
```

---

## 5. Toolchain Installation

Install `espup` with the stable Rust toolchain:

```bash
rustup run stable cargo install espup
espup install
```

Load the ESP environment in each new terminal:

```bash
. "$HOME/export-esp.sh"
```

Install the flashing utility:

```bash
rustup run stable cargo install espflash --locked
```

Verify the installation:

```bash
rustup toolchain list
espflash --version
```

---

## 6. Build

From the repository root:

```bash
cd rust_firmware
. "$HOME/export-esp.sh"
cargo build --release --bin rust_firmware
```

The application image is produced at:

```text
rust_firmware/target/xtensa-esp32-none-elf/release/rust_firmware
```

Run the host-side unit tests separately with stable Rust:

```bash
rustup run stable cargo test \
  --locked \
  --offline \
  --target x86_64-unknown-linux-gnu \
  --lib
```

---

## 7. Flash

Keep servo power off and USB connected. From `rust_firmware/`:

```bash
espflash flash \
  --chip esp32 \
  --port /dev/ttyUSB0 \
  target/xtensa-esp32-none-elf/release/rust_firmware
```

A successful operation ends with:

```text
Flashing has completed!
```

Verify that the CH340 port has returned:

```bash
test -c /dev/ttyUSB0 && echo "Serial port ready"
```

The firmware intentionally prints no normal logs on UART0. Text output would
corrupt the binary servo stream, so an empty serial terminal is expected.

---

## 8. Host Serial Configuration

Configure the host port as follows:

| Parameter | Value |
| --------- | ----- |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| Mode | Binary/raw |

Before a transaction, the host should discard stale local input bytes. For a
read command, it should wait up to 50 ms for the first response byte and
validate the complete response frame.

---

## 9. Binary Protocol

### 9.1 Frame format

```text
┌──────────┬────┬────────┬─────────┬────────────┬──────────┐
│ 55 55    │ ID │ Length │ Command │ Parameters │ Checksum │
│ 2 bytes  │ 1  │ 1      │ 1       │ 0..N       │ 1        │
└──────────┴────┴────────┴─────────┴────────────┴──────────┘
```

`Length` counts bytes from the `Length` field through the checksum:

```text
physical frame size = Length + 3
```

The checksum is:

```text
checksum = ~(ID + Length + Command + Parameters) & 0xFF
```

The firmware detects frame boundaries but does not alter bytes, verify the
checksum or interpret the command.

### 9.2 Safe position read

Read position from servo 1:

```text
55 55 01 03 1C DF
```

Example validated response:

```text
55 55 01 05 1C EC 01 F0
```

The two position bytes are signed little-endian. In this example:

```text
EC 01 = 492
```

### 9.3 Motion command

Move servo 1 to position 500 over 1000 ms:

```text
55 55 01 07 01 F4 01 E8 03 16
```

Motion writes do not produce an acknowledgement. Confirm a completed movement
with a later position read.

---

## 10. Recovery Behaviour

The firmware always starts in RX mode.

If a host frame is interrupted, it is discarded after 50 ms without a new
byte. The firmware then accepts the next `55 55` header. UART receive errors,
invalid lengths and discarded stale bytes do not stop the main loop.

After closing and reopening the host serial port, send a new complete frame;
no ESP32 reset or textual synchronization sequence is required.

---

## 11. Diagnostic Counters

UART0 cannot carry logs alongside binary servo traffic. The release ELF keeps
a debugger-visible symbol named `FIRMWARE_COUNTERS`.

Verify that it exists:

```bash
. "$HOME/export-esp.sh"
xtensa-esp32-elf-nm -S \
  target/xtensa-esp32-none-elf/release/rust_firmware \
  | grep FIRMWARE_COUNTERS
```

The symbol contains ten consecutive 32-bit counters:

| Index | Counter |
| ----: | ------- |
| 0 | Host UART RX errors |
| 1 | Host UART TX errors |
| 2 | Servo UART RX errors |
| 3 | Servo UART TX errors |
| 4 | Discarded stale servo bytes |
| 5 | Invalid host lengths |
| 6 | Oversized host frames |
| 7 | Incomplete host frame timeouts |
| 8 | Frames forwarded to UART2 |
| 9 | Servo bytes forwarded to UART0 |

Counters saturate at `u32::MAX` and are never emitted on the serial transport.

---

## 12. Restore the Stock MicroPython Firmware

Restoration erases the complete ESP32 flash. Keep servo power off throughout
the operation.

From the repository root:

```bash
cd micro_python_firmware
md5sum -c flash_dump_4mb.bin.md5
esptool --port /dev/ttyUSB0 --baud 921600 erase-flash
esptool --port /dev/ttyUSB0 --baud 921600 \
  write-flash 0x0 flash_dump_4mb.bin
esptool --port /dev/ttyUSB0 --baud 921600 \
  verify-flash 0x0 flash_dump_4mb.bin
```

Do not interrupt power during `write-flash` or `verify-flash`.

---

## 13. Troubleshooting

### 13.1 Serial port not found

- Check the USB cable and `lsusb` output.
- Confirm that the CH340 appears as `1a86:7523`.
- Inspect `/dev/ttyUSB*` and the stable `/dev/serial/by-id/` path.

### 13.2 Permission denied

- Check membership of `dialout` with `id`.
- Reopen the Linux login session after `usermod`.
- Use a temporary ACL only when needed.

### 13.3 No servo response

- Confirm that main servo power is on.
- Verify 115200 8N1 raw serial configuration.
- Check the servo ID and checksum.
- Enforce the 50 ms first-byte timeout on the host.
- Ensure only one response-producing request is in flight.

### 13.4 Text or boot bytes received

The ESP32 ROM may print boot information during reset. Flush host input after
opening the serial port and before the first transaction. The Rust application
itself emits no normal text logs.

### 13.5 Restore after a failed flash

Keep servo power off, reconnect USB, enter the ESP32 bootloader if required and
repeat either the Rust flash procedure or the stock restoration procedure.
