# UWB_RX_Test.cpp - Simple UWB Receiver Test

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

`UWB_RX_Test.cpp` implements a continuous UWB frame receiver using polling-based reception. This example demonstrates basic receive functionality by activating the receiver, polling for incoming frames, and printing received data to serial output.

## Purpose

- Demonstrate basic UWB frame reception
- Validate RX configuration parameters
- Test polling-based frame detection
- Display received frame data for debugging
- Monitor reception errors and timeouts

## Hardware Requirements

- ESP32-S3 microcontroller
- DW1000 UWB transceiver module
- Complete SPI and GPIO connections (see UWB_Init_Test.md)
- Transmitter device running compatible configuration (e.g., UWB_TX_Test)

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

**Alternative Configuration** (commented out in code):
```cpp
// Maximum range configuration with longer preamble and lower data rate
DWT_PLEN_4096, DWT_PAC64, DWT_BR_110K
```

### Antenna Delay

```cpp
#define TX_ANT_DLY 16436  // Transmit antenna delay (64 MHz PRF)
#define RX_ANT_DLY 16436  // Receive antenna delay (64 MHz PRF)
```

These values compensate for signal propagation delays through antenna circuitry. Default values provided; calibration required for accurate ranging.

## Operation Flow

### Setup Phase
1. Initialize serial communication (115200 baud)
2. Blink LED indicator
3. Deselect IMU on shared SPI bus
4. Configure and initialize SPI interface
5. Configure GPIO pins (RST, IRQ)
6. Perform hardware reset with SPI bug fix
7. Initialize DW1000 with microcode loading
8. Switch to high-speed SPI (16 MHz)
9. Apply configuration parameters
10. Set antenna delay values

### Main Loop
1. **Clear RX Buffer** - Reset local buffer to zeros
2. **Enable Receiver** - Call `dwt_rxenable(DWT_START_RX_IMMEDIATE)`
3. **Poll Status Register** - Wait for `SYS_STATUS_RXFCG` (good frame) or error flags
4. **Timeout Monitoring** - 2-second timeout if no frame received
5. **Process Frame** - On successful reception:
   - Read frame length from RX_FINFO register
   - Copy frame data to local buffer
   - Print frame data in hexadecimal
   - Display frame length
6. **Error Handling** - Detect and report RX errors
7. **Clear Status** - Reset status register for next reception
8. **Repeat** - Immediately restart receiver

## Status Register Monitoring

The receiver monitors these status flags:

### Success Flag
- `SYS_STATUS_RXFCG` (0x00002000) - RX Frame Control Good - Valid frame received with correct CRC

### Error Flags (SYS_STATUS_ALL_RX_ERR)
- `SYS_STATUS_RXPHE` - RX PHY Header Error
- `SYS_STATUS_RXFCE` - RX Frame Check Error (CRC failure)
- `SYS_STATUS_RXRFSL` - RX Reed-Solomon Frame Sync Loss
- `SYS_STATUS_RXSFDTO` - RX SFD Timeout
- `SYS_STATUS_AFFREJ` - Automatic Frame Filtering Rejection
- `SYS_STATUS_LDEERR` - Leading Edge Detection Error

## Frame Buffer

```cpp
#define FRAME_LEN_MAX 127  // Maximum 802.15.4 frame length
static uint8 rx_buffer[FRAME_LEN_MAX];
```

## Key Functions Used

### Platform Functions
- `dw1000_spi_init()` - Initialize SPI
- `dw1000_gpio_init()` - Configure GPIOs
- `dw1000_hard_reset()` - Reset DW1000
- `dw1000_spi_fix_bug()` - Apply SPI workaround
- `spi_set_rate_high()` - High-speed SPI mode

### DW1000 Driver Functions
- `dwt_initialise(DWT_LOADUCODE)` - Initialize with microcode
- `dwt_configure()` - Apply configuration
- `dwt_setrxantennadelay()` - Set RX antenna delay
- `dwt_settxantennadelay()` - Set TX antenna delay
- `dwt_rxenable()` - Enable receiver
- `dwt_read32bitreg()` - Read status registers
- `dwt_readrxdata()` - Read received frame data
- `dwt_write32bitreg()` - Clear status flags

## Expected Serial Output

### Successful Reception
```
========================================
             RX reader
========================================

IMU CS set to HIGH
SPI initialized
GPIO initialized
DW1000 reset complete
DW1000 initialized
DW1000 configured

========================================
   Initialization Complete!
   Waiting for messages...
========================================

RX Event - Status register: 0x00002000
Frame received - Length: 11 bytes
RX Data: C5 00 DE CA 01 23 45 67 89 AB CD
```

### Timeout
```
RX Timeout
RX Event - Status register: 0x00000000
```

### RX Error
```
RX Event - Status register: 0x00004000
RX Error detected
Error flags: 0x00004000
```

## Communication Requirements

To receive frames successfully:

1. **Matching Configuration** - Transmitter must use identical:
   - Channel number
   - PRF (pulse repetition frequency)
   - Preamble code
   - Data rate
   - SFD type

2. **Compatible Frame Format** - Transmitter should send valid 802.15.4 frames

3. **Range** - Devices must be within communication range (depends on configuration)

## Performance Notes

### Polling vs. Interrupts
- This example uses **polling mode** - CPU continuously checks status register
- More CPU-intensive than interrupt-driven reception
- Simpler to implement and debug
- See `UWB_RX_Send_Test.cpp` for interrupt-based alternative

### Timeout Implementation
```cpp
int64_t t_start = esp_timer_get_time();
while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
         (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
    if (esp_timer_get_time() - t_start > 2000000) { // 2 seconds
        Serial.println("RX Timeout");
        t_start = esp_timer_get_time();
    }
    vTaskDelay(1);  // Yield to other tasks
}
```

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| No frames received | TX/RX config mismatch | Verify channel, PRF, preamble codes match |
| Continuous timeouts | Out of range | Move devices closer together |
| RX errors (RXFCE) | Poor signal quality | Reduce distance, check antenna connections |
| SFD timeout | Weak preamble detection | Increase preamble length or PAC size |
| Buffer overflow | Frame > 127 bytes | Check transmitter frame length |

## Related Examples

- `UWB_TX_Test.cpp` - Polling-based transmitter (pair with this)
- `UWB_RX_Send_Test.cpp` - Interrupt-based RX with auto-response
- `UWB_DS_TWR_Resp_Test.cpp` - Double-sided ranging responder
- Official DecaWave: `ex_02a_simple_rx`

## Modifications

### Enable Long-Range Mode
Uncomment the alternative configuration:
```cpp
// static dwt_config_t dw1000_config = {
//     DWT_PLEN_4096,   /* 4096-symbol preamble */
//     DWT_PAC64,       /* 64-symbol PAC */
//     DWT_BR_110K,     /* 110 kbps data rate */
//     (4096 + 16 + 64 - 64) /* Adjusted SFD timeout */
// };
```

**Trade-offs:**
- ✅ Increased range and reliability
- ❌ Lower data rate (110 kbps vs 850 kbps)
- ❌ Longer preamble detection time
- ❌ Reduced power efficiency

## Notes

- Frame data is printed in hexadecimal format with spaces
- Status register is displayed before and after processing
- The receiver immediately restarts after each frame
- No frame filtering is applied - all valid frames are received
- CRC is automatically checked by DW1000 hardware
