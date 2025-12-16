# UWB_TX_Test.cpp - Simple UWB Transmitter Test

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

`UWB_TX_Test.cpp` implements a continuous UWB frame transmitter using polling-based transmission confirmation. This example transmits 802.15.4e standard blink frames with incrementing sequence numbers at 200ms intervals.

## Purpose

- Demonstrate basic UWB frame transmission
- Validate TX configuration parameters
- Test polling-based transmission confirmation
- Transmit test frames with incrementing sequence numbers
- Monitor transmission status through register polling

## Hardware Requirements

- ESP32-S3 microcontroller
- DW1000 UWB transceiver module
- Complete SPI and GPIO connections (see UWB_Init_Test.md)
- Receiver device to verify transmission (e.g., UWB_RX_Test)

## Configuration

### DW1000 Configuration Parameters

```cpp
static dwt_config_t dw1000_config = {
    1,                 /* Channel 1 (3.4944 GHz center frequency). */
    DWT_PRF_64M,       /* 64 MHz pulse repetition frequency. */
    DWT_PLEN_128,      /* 128-symbol preamble length. */
    DWT_PAC8,          /* Preamble acquisition chunk size (8). */
    9,                 /* TX preamble code 9. */
    9,                 /* RX preamble code 9. */
    1,                 /* Use non-standard SFD. */
    DWT_BR_850K,       /* Data rate 850 kbps. */
    DWT_PHRMODE_STD,   /* Standard PHY header mode. */
    (128 + 8 + 64 - 8) /* SFD timeout calculation. */
};
```

**Alternative Configuration** (commented out):
```cpp
// Maximum range: 4096-symbol preamble, PAC64, 110 kbps data rate
```

### Transmission Timing

```cpp
#define TX_DELAY_MS 200  // 200 milliseconds between frames
```

## Frame Structure

### 802.15.4e Standard Blink Frame

```cpp
static uint8 tx_msg[] = {
    0xC5,       // [0] Frame type: Blink frame identifier
    0x00,       // [1] Sequence number (increments each transmission)
    0xDE,       // [2-8] Message payload (arbitrary data)
    0xCA, 
    0x01, 
    0x23, 
    0x45, 
    0x67, 
    0x89, 
    0x00,       // [9-10] Frame Check Sequence (FCS/CRC)
    0x00        //        Automatically calculated by DW1000
};
```

**Frame Fields:**
- **Byte 0 (0xC5)**: Frame Control - identifies as blink frame
- **Byte 1**: Sequence number - increments 0→255, wraps to 0
- **Bytes 2-8**: Payload data (0xDECA012345678 9)
- **Bytes 9-10**: CRC - automatically computed and appended by DW1000 hardware

### Sequence Number

```cpp
#define BLINK_FRAME_SN_IDX 1  // Array index of sequence number
tx_msg[BLINK_FRAME_SN_IDX]++;  // Increment after each transmission
```

## Operation Flow

### Setup Phase
1. Initialize serial communication (115200 baud)
2. Blink LED indicator (3 times, 500ms)
3. Deselect IMU on shared SPI bus
4. Configure and initialize SPI interface
5. Configure GPIO pins (RST, IRQ)
6. Perform hardware reset with SPI bug fix
7. Initialize DW1000 with microcode
8. Switch to high-speed SPI (16 MHz)
9. Apply configuration parameters

### Main Loop (Transmission Cycle)

1. **Write Frame Data**
   ```cpp
   dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
   ```
   - Copies frame to DW1000 TX buffer
   - Zero offset means start at beginning of buffer

2. **Configure Frame Control**
   ```cpp
   dwt_writetxfctrl(sizeof(tx_msg), 0, 0);
   ```
   - Sets frame length (11 bytes)
   - No offset, no ranging bit set

3. **Print Pre-TX Status**
   - Reads and displays SYS_STATUS register before transmission

4. **Start Transmission**
   ```cpp
   dwt_starttx(DWT_START_TX_IMMEDIATE);
   ```
   - Begins immediate frame transmission
   - Returns DWT_SUCCESS or error code

5. **Poll for Completion**
   ```cpp
   while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {};
   ```
   - Continuously reads status register
   - Waits for `SYS_STATUS_TXFRS` (TX Frame Sent) flag

6. **Print Post-TX Status**
   - Displays status register showing TXFRS flag set

7. **Clear TX Event**
   ```cpp
   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
   ```
   - Clears TXFRS flag for next transmission

8. **Print Cleared Status**
   - Confirms status register cleared

9. **Inter-Frame Delay**
   ```cpp
   vTaskDelay(pdMS_TO_TICKS(TX_DELAY_MS));
   ```
   - 200ms delay before next transmission
   - Yields to FreeRTOS scheduler

10. **Increment Sequence Number**
    ```cpp
    tx_msg[BLINK_FRAME_SN_IDX]++;
    ```
    - Increments (modulo 256) for next frame

## Status Register Monitoring

### Key Status Flag

- **SYS_STATUS_TXFRS (0x00000080)** - TX Frame Sent
  - Set by hardware when frame transmission completes
  - Must be cleared by software before next transmission

### Status Register Values (Example)

```
TX started - Status register: 0x00000000
TX event detected - Status register: 0x00000080
TX event cleared - Status register: 0x00000000
```

## Key Functions Used

### Platform Functions
- `dw1000_spi_init()` - Initialize SPI interface
- `dw1000_gpio_init()` - Configure GPIO pins
- `dw1000_hard_reset()` - Reset DW1000
- `dw1000_spi_fix_bug()` - Apply SPI workaround
- `spi_set_rate_high()` - High-speed SPI mode

### DW1000 Driver Functions
- `dwt_initialise(DWT_LOADUCODE)` - Initialize with microcode
- `dwt_configure()` - Apply configuration
- `dwt_writetxdata()` - Write frame data to TX buffer
- `dwt_writetxfctrl()` - Configure frame control
- `dwt_starttx()` - Start frame transmission
- `dwt_read32bitreg()` - Read status register
- `dwt_write32bitreg()` - Clear status flags

## Expected Serial Output

```
========================================
              TX sender
========================================

IMU CS set to HIGH
SPI initialized
GPIO initialized
DW1000 reset complete
DW1000 initialized
DW1000 configured

========================================
   Initialization Complete!
   Starting TX...
========================================

TX started - Status register: 0x00000000
TX event detected - Status register: 0x00000080
TX event cleared - Status register: 0x00000000

TX started - Status register: 0x00000000
TX event detected - Status register: 0x00000080
TX event cleared - Status register: 0x00000000

[Repeats every 200ms...]
```

## Transmission Rate

- **Frame interval**: 200ms
- **Frames per second**: 5 FPS
- **Frame length**: 11 bytes
- **Effective data rate**: 55 bytes/sec (440 bits/sec)

**Note**: Air data rate is 850 kbps, but actual throughput limited by inter-frame delay.

## Performance Notes

### Polling vs. Interrupts
- This example uses **polling mode** - CPU waits for TX completion
- Simple and deterministic
- CPU blocked during transmission (~120-500µs depending on preamble)
- See `UWB_TX_ISR_Test.cpp` for interrupt-based alternative
- See `UWB_TX_Resp_Test.cpp` for full-duplex TX+RX with interrupts

### Transmission Time Estimate

With current configuration:
- Preamble: 128 symbols @ 64 MHz PRF ≈ 130 µs
- Data: 11 bytes @ 850 kbps ≈ 103 µs
- **Total air time**: ~233 µs per frame

## Verification

### Using UWB_RX_Test

1. Upload `UWB_TX_Test.cpp` to one ESP32-S3 (Transmitter)
2. Upload `UWB_RX_Test.cpp` to another ESP32-S3 (Receiver)
3. Power both devices
4. Receiver should display:
   ```
   Frame received - Length: 11 bytes
   RX Data: C5 00 DE CA 01 23 45 67 89 XX XX
   ```
5. Sequence number (byte [1]) should increment: 00→01→02→...→FF→00

### LED Oscilloscope Verification

Connect oscilloscope to UWB antenna output to observe:
- Periodic RF bursts every 200ms
- Burst duration ~230µs
- Preamble and data portions visible with high bandwidth scope

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| `dwt_writetxdata failed` | Buffer size error | Check `sizeof(tx_msg)` ≤ 125 bytes |
| `dwt_starttx failed` | Device in wrong state | Ensure initialization completed successfully |
| Status register never shows TXFRS | Hardware failure | Check power supply, SPI connections, device ID |
| Receiver doesn't receive | Config mismatch | Verify channel, PRF, preamble codes match RX |
| Transmission too slow/fast | Timing error | Adjust `TX_DELAY_MS` value |

## Modifications

### Change Transmission Rate

```cpp
#define TX_DELAY_MS 1000  // 1 frame per second
// or
#define TX_DELAY_MS 50    // 20 frames per second
```

**Minimum delay**: ~1ms (limited by frame air time + processing)

### Custom Payload

```cpp
static uint8 tx_msg[] = {
    0xC5, 0x00,           // Frame control + sequence
    'H', 'E', 'L', 'L', 'O',  // Custom ASCII message
    0x00, 0x00            // CRC (auto-calculated)
};
```

### Enable Long-Range Mode

Uncomment alternative configuration:
- Increases range significantly
- Reduces data rate to 110 kbps
- Longer transmission time (~3-4ms per frame)

## Related Examples

- `UWB_RX_Test.cpp` - Polling-based receiver (pair with this)
- `UWB_TX_ISR_Test.cpp` - Interrupt-based TX confirmation
- `UWB_TX_Resp_Test.cpp` - TX with response waiting (full duplex)
- Official DecaWave: `ex_01a_simple_tx`

## Notes

- CRC is automatically computed by DW1000 hardware - don't manually calculate
- Frame transmission is atomic - cannot be interrupted once started
- Sequence number wraps at 256 (8-bit counter)
- No acknowledgment or retransmission - fire-and-forget transmission
- Compatible with IEEE 802.15.4-2011 standard blink frames
