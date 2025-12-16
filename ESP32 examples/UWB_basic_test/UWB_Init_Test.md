# UWB_Init_Test.cpp - DW1000 Initialization Test

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

`UWB_Init_Test.cpp` is a basic initialization test program for the DW1000 UWB transceiver on ESP32-S3. This example validates hardware connections and basic device functionality by performing a complete initialization sequence and reading the device ID.

## Purpose

- Verify SPI communication between ESP32-S3 and DW1000
- Test GPIO configuration for control signals (RST, IRQ, CS)
- Validate device initialization and microcode loading
- Confirm device identity by reading the DW1000 device ID register

## Hardware Requirements

- ESP32-S3 microcontroller
- DW1000 UWB transceiver module
- SPI connection (MOSI, MISO, CLK, CS)
- GPIO connections (RST, IRQ)
- Optional: IMU on same SPI bus (CS pin is set high to deselect)

## Configuration

### DW1000 Configuration Parameters

```cpp
static dwt_config_t dw1000_config = {
    5,                /* Channel number. */
    DWT_PRF_16M,      /* Pulse repetition frequency (16 MHz). */
    DWT_PLEN_256,     /* Preamble length (256 symbols). */
    DWT_PAC16,        /* Preamble acquisition chunk size. */
    3,                /* TX preamble code. */
    3,                /* RX preamble code. */
    0,                /* Standard SFD. */
    DWT_BR_850K,      /* Data rate (850 kbps). */
    DWT_PHRMODE_STD,  /* Standard PHY header mode. */
    (256 + 1 + 8 - 8) /* SFD timeout calculation. */
};
```

### Pin Configuration

Pins are defined in `HardwareDefs.hpp`:
- `SPI_MOSI_PIN` - SPI Master Out Slave In
- `SPI_MISO_PIN` - SPI Master In Slave Out
- `SPI_CLK_PIN` - SPI Clock
- `UWB_CS_PIN` - DW1000 Chip Select
- `UWB_RST_PIN` - DW1000 Reset
- `UWB_IRQ_PIN` - DW1000 Interrupt Request
- `IMU_CS_PIN` - IMU Chip Select (deselected during test)

## Initialization Sequence

1. **Serial Communication** - Initialize UART at 115200 baud
2. **LED Indicator** - Blink LED 3 times (500ms intervals)
3. **IMU Deselection** - Set IMU CS pin high to prevent conflicts
4. **SPI Bus Configuration** - Configure SPI2 with 1024-byte max transfer
5. **DW1000 SPI Initialization** - Initialize SPI device handle
6. **GPIO Configuration** - Setup RST and IRQ pins
7. **Hardware Reset** - Assert/deassert RST pin
8. **SPI Bug Fix** - Apply post-reset SPI workaround
9. **Device Initialization** - Load microcode with `dwt_initialise(DWT_LOADUCODE)`
10. **SPI Speed Increase** - Switch from low (2 MHz) to high (16 MHz) speed
11. **Device Configuration** - Apply configuration parameters
12. **Device ID Read** - Verify communication by reading device ID

## Expected Device ID

DW1000 should return device ID: `0xDECA0130`
- `0xDECA` - DecaWave manufacturer identifier
- `0x01` - DW1000 device type
- `0x30` - Revision/version

## Key Functions Used

### Platform Functions (ESP32 Port)
- `dw1000_spi_init()` - Initialize SPI interface for DW1000
- `dw1000_gpio_init()` - Configure GPIO pins
- `dw1000_hard_reset()` - Perform hardware reset
- `dw1000_spi_fix_bug()` - Apply SPI workaround
- `spi_set_rate_high()` - Switch to high-speed SPI

### DW1000 Driver Functions
- `dwt_initialise()` - Initialize device and load microcode
- `dwt_configure()` - Apply configuration parameters
- `dwt_readdevid()` - Read 32-bit device identifier

## Error Handling

The program halts with error messages if:
- SPI initialization fails
- GPIO initialization fails
- Device initialization returns `DWT_ERROR`

## Usage

1. Connect DW1000 module to ESP32-S3 according to pin definitions
2. Upload this program to ESP32-S3
3. Open Serial Monitor at 115200 baud
4. Observe initialization sequence messages
5. Verify device ID matches expected value `0xDECA0130`

## Expected Serial Output

```
========================================
   DW1000 Simplified Init Test
========================================

IMU CS set to HIGH
SPI initialized
GPIO initialized
DW1000 reset complete
Calling dwt_initialise(DWT_LOADUCODE)...
SUCCESS: dwt_initialise passed
SPI speed set to high (16 MHz)
DW1000 configured

Device ID: 0xDECA0130

========================================
   Initialization Complete!
========================================
```

## Common Issues

1. **SPI init failed** - Check SPI pin connections and ensure pins are not already in use
2. **dwt_initialise failed** - Verify DW1000 power supply (3.3V), reset pin connection, and SPI wiring
3. **Incorrect Device ID** - Check SPI clock polarity/phase, wire lengths, and signal integrity

## Related Examples

- `UWB_TX_Test.cpp` - Simple transmitter after initialization
- `UWB_RX_Test.cpp` - Simple receiver after initialization
- Official DecaWave example: `ex_01a_simple_tx`

## Notes

- This example uses polling mode (no interrupts)
- After initialization, the device enters IDLE state
- The `loop()` function does nothing - this is purely an initialization test
- IMU CS pin must be high to prevent SPI bus conflicts if sharing the bus
