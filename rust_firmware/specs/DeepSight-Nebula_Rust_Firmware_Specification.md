# DeepSight-Nebula — Rust Firmware Specification

**Status:** Implemented, pending performance validation
**Version:** 0.2
**Date:** 2026-08-23

---

## 1. Context & Objectives

### 1.1 Project context

DeepSight-Nebula is a robotics project built around a Hiwonder xArm controlled by an ESP32.

The robot is controlled from a host computer running the higher-level software stack, including ROS 2 and motion planning. The ESP32 provides the low-level communication interface with the arm's servo bus.

The current firmware is implemented in MicroPython.

With the stock firmware, the host communicates with the ESP32 through a USB serial connection using the MicroPython REPL. The firmware interprets the received Python commands, constructs the binary servo protocol frames, sends them through UART2, receives servo responses when applicable, parses them, and returns Python-level values to the host.

The communication chain is therefore:

```text
Host PC
   │
   │ USB serial (CH340 / UART0) / MicroPython REPL
   │ textual Python commands
   ▼
ESP32 / MicroPython
   │
   │ binary servo protocol
   ▼
UART2
   │
   ▼
Servo bus
```

### 1.2 Motivation

The MicroPython layer introduces processing and timing overhead in the communication path.

In particular, the current implementation performs byte-level UART reception in software. During the benchmark performed on 2026-08-22, approximately 23.6% of position reads failed due to either a timeout or a corrupted response frame.

The Rust firmware is intended to remove this additional software layer and provide a lightweight, deterministic communication bridge between the host and the servo bus.

### 1.3 Objectives

The Rust firmware shall:

- replace the current MicroPython firmware for servo communication;
- expose the servo bus to the host through the CH340/UART0 serial port;
- forward binary data between UART0 and UART2;
- correctly control the half-duplex direction of the servo bus;
- minimize latency and jitter;
- provide reliable buffered UART reception;
- preserve the binary data exchanged with the servo bus.

### 1.4 Scope

The firmware is a low-level communication component.

It is **not** responsible for:

- ROS 2;
- MoveIt 2;
- trajectory planning;
- inverse kinematics;
- object detection;
- robot-level motion planning;
- generating servo protocol commands from high-level robot commands.

The host-side software is responsible for constructing and interpreting the servo protocol.

---

## 2. Architecture

### 2.1 System architecture

The Rust firmware is positioned between the host computer and the servo bus.

```text
┌──────────────────────────────────────┐
│              Host PC                 │
│                                      │
│ ROS 2 / MoveIt 2 / Driver            │
│                                      │
│ Servo protocol encoding/decoding     │
└──────────────────┬───────────────────┘
                   │
           USB serial / UART0
                   │
                   ▼
┌──────────────────────────────────────┐
│         ESP32 Rust Firmware           │
│                                      │
│ UART0 ↔ UART2 transport              │
│ Half-duplex bus direction control    │
│ Communication buffering              │
└──────────────────┬───────────────────┘
                   │
                 UART2
                   │
                   ▼
┌──────────────────────────────────────┐
│             Servo Bus                │
│                                      │
│ Servo 1 ... Servo 5                  │
└──────────────────────────────────────┘
```

### 2.2 Responsibility separation

#### Host

The host is responsible for:

- generating servo command frames;
- calculating command checksums;
- sending commands;
- receiving response frames;
- validating response frames;
- decoding response parameters;
- exposing robot-level functionality to ROS 2.

#### Rust firmware

The firmware is responsible for:

- host UART0 communication through the CH340;
- UART2 communication;
- forwarding bytes between UART0 and UART2;
- controlling TX/RX direction on the half-duplex bus;
- buffering communication data;
- handling transport-level communication failures.

#### Servos

The servos are responsible for:

- executing received commands;
- generating responses to read commands;
- maintaining their own state.

### 2.3 Design principle

The firmware shall remain as close as possible to a transparent transport layer.

A binary sequence received from UART0 shall be transmitted to UART2 without protocol-level modification.

A binary sequence received from UART2 shall be transmitted to UART0 without protocol-level modification.

The firmware therefore does not replace the servo protocol with a new application protocol.

---

## 3. Hardware & Communication Interface

### 3.1 ESP32

The firmware runs on the ESP32 used by the xArm controller.

The firmware uses:

- UART0 through the board's CH340 USB-to-serial converter for communication with the host;
- UART2 for communication with the servo bus;
- GPIOs for controlling the direction of the half-duplex bus.

### 3.2 UART configuration

The servo bus uses UART2 with the following configuration:

| Parameter     | Value       |
| ------------- | ----------- |
| UART          | UART2       |
| Baud rate     | 115200 baud |
| Data bits     | 8           |
| Parity        | None        |
| Stop bits     | 1           |
| Communication | Half-duplex |
| TX            | GPIO 26     |
| RX            | GPIO 35     |

### 3.3 Bus direction control

The direction of the half-duplex bus is controlled using:

| Signal  | GPIO | Function            |
| ------- | ---: | ------------------- |
| `tx_en` |   25 | Enable transmission |
| `rx_en` |   12 | Enable reception    |

Both signals are active high. The levels recovered from the stock MicroPython
firmware and validated on the physical controller are:

| Mode | GPIO25 `tx_en` | GPIO12 `rx_en` |
| ---- | -------------: | -------------: |
| RX   |              0 |              1 |
| TX   |              1 |              0 |

GPIO12 is an ESP32 strapping pin. The controller already uses it for `rx_en`
in the stock firmware. The Rust firmware only drives it after reset strapping
has been sampled and configures TX inactive before enabling RX.

When a command is transmitted, the bus must be configured for TX.

Immediately after the last byte of the command has been transmitted, the firmware must switch the bus back to RX.

This transition is critical for read commands because the addressed servo sends its response on the same physical bus.

```text
TX enabled
    │
    │ transmit complete command
    ▼
last byte transmitted
    │
    │ immediate direction switch
    ▼
RX enabled
    │
    │ receive servo response
    ▼
response available
```

### 3.4 Host serial interface

The ESP32-WROOM-32D has no native USB controller. The physical USB connector
on the controller is connected to a QinHeng CH340 USB-to-serial converter
(`1a86:7523`), which exposes ESP32 UART0 to the host.

UART0 uses 115200 baud, 8 data bits, no parity, one stop bit and no flow
control. The serial data path is binary.

The firmware shall not require textual commands, Python expressions, JSON, or another serialization format.

The host sends the binary servo protocol directly.

---

## 4. Servo Communication Protocol

### 4.1 Frame format

Command and response frames use the same binary structure:

```text
┌──────────┬──────────┬────────┬─────────┬────────────┬──────────┐
│ Header   │ Servo ID │ Length │ Command │ Parameters │ Checksum │
│ 2 bytes  │ 1 byte   │ 1 byte │ 1 byte  │ 0..4 bytes │ 1 byte   │
└──────────┴──────────┴────────┴─────────┴────────────┴──────────┘
```

### 4.2 Header

Every frame starts with:

```text
0x55 0x55
```

### 4.3 Servo ID

The servo ID occupies one byte.

The current arm uses servo IDs `0x01` through `0x05`.

In a response frame, the field contains the ID of the servo that generated the response.

### 4.4 Length

The `Length` field specifies the number of bytes from the `Length` field through the checksum, inclusive.

Therefore:

```text
Length = 3 + number_of_parameters
```

The total frame size on the wire is:

```text
Total frame size = Length + 3
```

### 4.5 Command

The command field contains the servo protocol command code.

A response frame echoes the command code of the corresponding request.

### 4.6 Parameters

Parameters depend on the command.

A maximum of four parameter bytes is currently used by the analysed protocol.

### 4.7 Checksum

The checksum is calculated over:

```text
Servo ID + Length + Command + all parameter bytes
```

using:

```text
checksum = ~(sum) & 0xFF
```

The low byte of the sum is retained before applying the bitwise NOT.

### 4.8 Endianness

All multi-byte numerical values use little-endian byte order.

For example:

```text
500 = 0x01F4

encoded as:

F4 01
```

---

### 4.9 Host → Servo commands

The following commands have been identified through reverse engineering of the stock MicroPython firmware.

|   Code | Command                      | Parameters                        | Length |
| -----: | ---------------------------- | --------------------------------- | -----: |
| `0x01` | `SERVO_MOVE_TIME_WRITE`      | position, time                    |      7 |
| `0x02` | `SERVO_MOVE_TIME_READ`       | none                              |      3 |
| `0x0C` | `SERVO_MOVE_STOP`            | none                              |      3 |
| `0x0D` | `SERVO_ID_WRITE`             | new ID                            |      4 |
| `0x0E` | `SERVO_ID_READ`              | none                              |      3 |
| `0x11` | `SERVO_ANGLE_OFFSET_ADJUST`  | signed offset                     |      4 |
| `0x12` | `SERVO_ANGLE_OFFSET_WRITE`   | none                              |      3 |
| `0x13` | `SERVO_ANGLE_OFFSET_READ`    | none                              |      3 |
| `0x1B` | `SERVO_VIN_READ`             | none                              |      3 |
| `0x1C` | `SERVO_POS_READ`             | none                              |      3 |
| `0x1D` | `SERVO_OR_MOTOR_MODE_WRITE`  | mode, `0x00`, speed               |      7 |
| `0x1F` | `SERVO_LOAD_OR_UNLOAD_WRITE` | `1` = torque on, `0` = torque off |      4 |

#### `SERVO_MOVE_TIME_WRITE`

Parameters:

```text
position_low
position_high
time_low
time_high
```

Position is expressed in the servo protocol range `0..1000`.

Time is expressed in milliseconds.

Example: move servo `1` to position `500` in `1000 ms`:

```text
55 55 01 07 01 F4 01 E8 03 16
```

The Rust firmware does not generate this frame. The host constructs it and the firmware forwards the resulting bytes.

---

### 4.10 Servo → Host responses

Only READ commands generate responses.

WRITE commands are fire-and-forget and do not generate an acknowledgement.

The analysed response formats are:

|   Code | Request                   | Response parameters | Length | Value                    |
| -----: | ------------------------- | ------------------- | -----: | ------------------------ |
| `0x1C` | `SERVO_POS_READ`          | position            |      5 | signed `int16` LE        |
| `0x1B` | `SERVO_VIN_READ`          | voltage             |      5 | unsigned `uint16` LE, mV |
| `0x0E` | `SERVO_ID_READ`           | ID                  |      4 | `uint8`                  |
| `0x13` | `SERVO_ANGLE_OFFSET_READ` | offset              |      4 | signed `int8`            |
| `0x02` | `SERVO_MOVE_TIME_READ`    | position + time     |      7 | protocol-defined         |

### 4.11 Position response

`SERVO_POS_READ` returns a signed 16-bit little-endian value.

The raw value must be preserved when decoding.

The value is not guaranteed to be within `0..1000`. A mechanically forced servo may return a value outside its nominal range.

Example:

```text
55 55 02 05 1C F4 01 E7
```

This corresponds to servo `2` reporting position `500`.

### 4.12 Read / write behaviour

READ commands:

```text
Host ─────────► Servo
       request
Host ◄───────── Servo
       response
```

WRITE commands:

```text
Host ─────────► Servo
       request

       no response
```

The absence of an acknowledgement means that successful transmission of a WRITE command does not by itself confirm that the servo has reached the requested state.

For example, motion confirmation requires polling `SERVO_POS_READ`.

---

## 5. Firmware Behaviour

### 5.1 Startup

At startup, the firmware shall initialize:

1. the host UART0 interface;
2. UART2;
3. the half-duplex direction control;
4. the communication buffers.

Once initialization is complete, the firmware enters its normal communication state.

```text
BOOT
  │
  ▼
Initialize interfaces
  │
  ▼
READY
```

### 5.2 Host UART0 → servo UART2 transaction

When bytes are received from the host:

1. the bytes are received through the USB serial port and UART0;
2. the data is buffered as required;
3. the bus is switched to TX;
4. the bytes are transmitted through UART2;
5. after the last byte has been transmitted, the bus is switched immediately to RX.

The firmware shall not modify the received bytes.

```text
USB serial / UART0
 │
 ▼
RX buffer
 │
 ▼
TX mode
 │
 ▼
UART2
 │
 ▼
last byte
 │
 ▼
RX mode
```

### 5.3 Servo UART2 → host UART0 transaction

When bytes are received from the servo bus:

1. UART2 receives the bytes;
2. the bytes are buffered as required;
3. the data is transmitted through UART0 and the CH340;
4. the data is not modified.

```text
UART2
 │
 ▼
RX buffer
 │
 ▼
UART0 TX / CH340
 │
 ▼
Host
```

### 5.4 Read command sequence

A read operation follows the following communication sequence:

1. clear stale UART RX data;
2. switch the bus to TX;
3. transmit the read command;
4. switch the bus immediately to RX;
5. wait for the servo response;
6. receive the response bytes;
7. forward the response to the host.

The host is responsible for interpreting the response frame.

### 5.5 Write command sequence

A write operation follows:

1. switch the bus to TX;
2. transmit the complete command;
3. switch the bus back to RX;
4. complete the transaction.

No response is expected from the servo.

### 5.6 Response reception

The host waits up to 50 ms for the first response byte after a read request.
The firmware does not interpret the command code and therefore cannot decide
whether a response is expected. It continuously drains the buffered UART2 RX
FIFO and forwards every received byte to UART0 without modification.

### 5.7 Frame handling

The passthrough layer shall preserve the complete byte stream.

The firmware does not need to interpret command parameters or response values in order to forward them.

Protocol-level validation and decoding belong to the host-side driver unless explicitly required by a future firmware feature.

The firmware only recognizes physical command boundaries. It searches for
`55 55`, then reads the ID and `Length` fields. The physical frame size is
`Length + 3`. It neither validates the checksum nor interprets the command.

An incomplete host frame is discarded after 50 ms without a new byte. This
allows a new host process to recover after a disconnection in the middle of a
frame.

### 5.8 Transaction ownership

The host shall keep at most one request expecting a response in flight. The
firmware does not match requests with responses and does not generate an
additional transport envelope. Multiple fire-and-forget write frames may be
sent consecutively.

---

## 6. Firmware Capabilities

The firmware provides access to the capabilities of the servo protocol through the CH340/UART0 serial port.

### 6.1 Motion

The host can:

- command a servo to move to a position over a specified duration;
- stop a servo;
- read the current servo position;
- read the last commanded position and movement time.

### 6.2 Servo configuration

The host can:

- read a servo ID;
- write a new servo ID;
- read the angle offset;
- adjust the angle offset;
- save the current angle offset.

### 6.3 Servo state

The host can:

- read the servo input voltage;
- enable servo torque;
- disable servo torque;
- configure servo/motor mode.

### 6.4 Firmware capability boundary

These capabilities are provided by the underlying servo protocol.

They are not high-level commands implemented by the Rust firmware itself.

The firmware provides transport for these operations.

This distinction keeps the firmware independent from ROS 2 and from the robot's high-level control logic.

---

## 7. Error & Failure Behaviour

### 7.1 Host serial disconnection

The CH340 does not provide a native USB connection-state event to the ESP32
firmware. Consequently, the firmware detects transport interruption through
UART errors or expiration of an incomplete frame rather than through a USB
disconnect event. It remains in RX and accepts new frames when the host reopens
the serial port.

### 7.2 UART errors

UART communication errors shall not cause undefined firmware behaviour.

The communication path shall recover to a valid state after a UART error.

### 7.3 Buffer overflow

Buffers shall be bounded.

If the incoming data rate exceeds the available buffer capacity, the firmware shall detect the overflow and recover without corrupting the firmware state.

Overflows and malformed lengths increment debugger-visible diagnostic counters.

### 7.4 Read timeout

If a read command does not produce a response within the configured timeout, the host-side driver shall consider the read operation failed.

The current reference timeout is 50 ms.

### 7.5 Invalid response

The host-side protocol parser shall reject responses with:

- invalid header;
- invalid length;
- invalid checksum.

The firmware itself should forward the raw response bytes without altering them.

### 7.6 Communication loss

The firmware remains operational after a temporary communication failure and
resumes normal UART0 ↔ UART2 communication when the path becomes available.

On communication loss, the firmware:

- returns and remains in RX mode;
- discards an incomplete host frame after 50 ms;
- does not generate any new servo command;
- does not inject a stop command, because it has no authoritative list of
  moving servo IDs and must remain a transparent transport;
- allows a movement already accepted by a servo to finish according to the
  duration encoded by the host.

System-level emergency stopping is outside this transport firmware and must be
implemented through power control or a higher-level safety component.

### 7.7 Diagnostic counters

The release firmware retains the `FIRMWARE_COUNTERS` symbol containing
saturating 32-bit atomic counters for host and servo UART errors, discarded
stale bytes, malformed lengths, oversized frames, incomplete-frame timeouts,
forwarded frames and forwarded servo bytes.

The counters are readable through a debugger and are never printed on UART0,
because diagnostic text would corrupt the binary host transport.

---

## 8. Performance Requirements

### 8.1 Objective

The primary performance objective of the Rust firmware is to reduce the communication latency and failure rate introduced by the MicroPython implementation.

### 8.2 Reference baseline

The benchmark performed on 2026-08-22 measured approximately:

- 23.6% failed position reads with the stock firmware;
- failures caused by timeout or corrupted response frames;
- significant overhead from the MicroPython per-byte UART reception loop.

The Rust firmware shall eliminate the software behaviour responsible for this overhead.

### 8.3 Requirements

The firmware shall prioritize:

- low UART0 → UART2 latency;
- low UART2 → UART0 latency;
- minimal TX → RX direction-switch latency;
- buffered UART reception;
- low jitter;
- deterministic memory usage;
- absence of unnecessary per-byte delays.

### 8.4 Validation

The Rust firmware shall be benchmarked using the same or equivalent workload as the stock firmware.

The benchmark should measure at least:

- command-to-transmission latency;
- response reception latency;
- successful read rate;
- failed read rate;
- behaviour under repeated commands.

The target is to significantly improve upon the MicroPython baseline while maintaining reliable communication with the servo bus.
