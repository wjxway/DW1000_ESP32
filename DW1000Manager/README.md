# DW1000Manager Library

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently.

## Overview

High-level interface for UWB ranging using DW1000 with Double-Sided Two-Way Ranging (DS-TWR) protocol.

**Built on**: DWM1000_ESP32 library  
**Protocol**: IEEE 802.15.4 DS-TWR  
**Platform**: ESP32, ESP32-S3

## Key Features

- **Simple API**: Initialize(), Begin(), Stop(), IsActive() for each role
- **Automatic Management**: FreeRTOS task handles ranging at configurable intervals (Initiator) or continuous operation (Responder)
- **Internal Queue**: Responder manages its own queue (size 10, auto-overwrite oldest)
- **Pre-optimized**: Channel 1, 64 MHz PRF, 128 symbols preamble, 850 kbps
- **Shared Code**: Common timestamp and helper functions reduce duplication

## Quick Start

### Initiator

```cpp
#include <DW1000Manager.hpp>
#include <driver/spi_master.h>

void setup() {
    // Initialize SPI bus first
    spi_bus_config_t spi_cfg = {
        .mosi_io_num = 11,
        .miso_io_num = 13,
        .sclk_io_num = 12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024
    };
    spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    
    // Initialize UWB hardware (only device-specific pins needed)
    if (!UWBRanging::Initiator::Initialize(
            10,  // CS pin
            15,  // INT pin
            6))  // RST pin
    {
        // Handle error
    }
    
    // Start ranging every 20ms (50 Hz)
    UWBRanging::Initiator::Begin(20);
}

void loop() {
    // Ranging runs automatically
}
```

### Responder

```cpp
#include <DW1000Manager.hpp>
#include <driver/spi_master.h>

QueueHandle_t queue;

void setup() {
    // Initialize SPI bus first
    spi_bus_config_t spi_cfg = {
        .mosi_io_num = 11,
        .miso_io_num = 13,
        .sclk_io_num = 12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024
    };
    spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    
    // Initialize UWB and get queue handle (only device-specific pins needed)
    queue = UWBRanging::Responder::Initialize(
        10,  // CS pin
        15,  // INT pin
        6);  // RST pin
    if (queue == NULL) {
        // Handle error
    }
    
    // Start receiving
    UWBRanging::Responder::Begin();
}

void loop() {
    UWBRanging::RangingResult result;
    if (xQueueReceive(queue, &result, 0) == pdTRUE) {
        Serial.printf("Distance: %.3f m\n", result.distance_m);
    }
}
```

## API Reference

### Common Types

```cpp
struct RangingResult {
    double distance_m;              // Distance in meters
    uint64_t measurement_time_us;   // Timestamp (μs since boot)
};
```

### Initiator

| Function | Description |
|----------|-------------|
| `bool Initialize(cs_pin, int_pin, rst_pin, callback_priority=6, ranging_priority=1)` | Initialize UWB hardware with device pins |
| `void Begin(uint32_t interval_ms)` | Start ranging (0 = manual mode) |
| `void Stop()` | Stop ranging |
| `bool TriggerRanging()` | Manually trigger one ranging exchange |
| `bool IsActive()` | Check if initialized and running |

**Note**: SPI bus (SPI2_HOST) must be initialized before calling `Initialize()`.

**Task Priorities:** ISR task handles hardware interrupts; ranging task sends poll messages.

### Responder

| Function | Description |
|----------|-------------|
| `QueueHandle_t Initialize(cs_pin, int_pin, rst_pin, callback_priority=6)` | Initialize UWB hardware, returns queue handle |
| `void Begin()` | Start receiving ranging requests |
| `void Stop()` | Stop receiving |
| `bool IsActive()` | Check if initialized and running |

**Note**: SPI bus (SPI2_HOST) must be initialized before calling `Initialize()`.

## Hardware Setup

Pin configuration is passed to `Initialize()` function. Example wiring:

```
ESP32-S3          DW1000
--------          ------
GPIO 11  -------> MOSI
GPIO 13  <------- MISO
GPIO 12  -------> SCLK
GPIO 10  -------> CS
GPIO 6   -------> RST
GPIO 15  <------- IRQ
3.3V     -------> VDD
GND      -------> GND
```

**Important**: 
- DW1000 is 3.3V only!
- SPI bus must be initialized before calling DW1000Manager::Initialize()
- Pins are configurable via Initialize() parameters

## Performance

- **Recommended Rate**: 20-40ms interval (25-50 Hz)
- **Accuracy**: 2-5 cm std dev (typical, under good conditions)
- **Range**: 50-100 meters line of sight

## Configuration

Pre-configured constants (in `DW1000Manager.cpp`):

```cpp
// Antenna delays (64 MHz PRF)
constexpr uint16_t TX_ANT_DLY = 16436;
constexpr uint16_t RX_ANT_DLY = 16436;

// Timing (microseconds)
constexpr uint16_t POLL_TX_TO_RESP_RX_DLY_UUS = 900;
constexpr uint16_t RESP_RX_TO_FINAL_TX_DLY_UUS = 1000;
constexpr uint16_t RESP_RX_TIMEOUT_UUS = 2000;

// Queue
constexpr uint8_t QUEUE_SIZE = 10;
```

## Debugging

Enable debug output in `DW1000Manager.cpp`:

```cpp
#define DEBUG_INIT 1      // Initialization messages
#define DEBUG_RANGING 1   // Ranging messages
```

Common issues:
- **Init fails**: Check wiring, 3.3V power, RST/IRQ connections
- **No results**: Enable debug, verify both devices initialized, check distance < 1m initially
- **Low rate**: Verify `spi_set_rate_high()` called internally, check SPI bus conflicts
- **High variance**: Test in open area, calibrate antenna delays, check for metal objects

## Antenna Delay Calibration

For improved accuracy:

1. Measure known distance (e.g., 1.000m)
2. Calculate error: `error_m = measured - actual`
3. Adjust: `delay_adjustment = (error_m / actual) * 32768`
4. Update: `new_delay = 16436 + delay_adjustment`
5. Modify constants in `DW1000Manager.cpp`

## Examples

See [examples/README.md](examples/README.md) for complete example applications.

## Important Notes

### SPI Bus Initialization

**CRITICAL**: The SPI bus (SPI2_HOST) must be initialized **before** calling `Initialize()`. The library expects the SPI bus to be ready and will only configure the DW1000 device on that bus.

```cpp
// Step 1: Initialize SPI bus
spi_bus_config_t spi_cfg = {
    .mosi_io_num = 11,
    .miso_io_num = 13,
    .sclk_io_num = 12,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024
};
spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);

// Step 2: Initialize DW1000 device (only device pins needed)
UWBRanging::Initiator::Initialize(cs_pin, int_pin, rst_pin);
```

### Pin Parameters

The `Initialize()` function only requires **device-specific pins**:
- `cs_pin`: SPI chip select for DW1000
- `int_pin`: Interrupt pin from DW1000
- `rst_pin`: Reset pin for DW1000

SPI bus pins (MOSI, MISO, SCLK) are **not** needed since the bus is already initialized.

### Task Priorities

Optional parameters control FreeRTOS task priorities:
- **Initiator**: `callback_priority` (default 6), `ranging_priority` (default 1)
- **Responder**: `callback_priority` (default 6)

Higher priority values = higher priority tasks.

## Migration from Previous Versions

| Previous API | Current API |
|--------------|-------------|
| Hardware pins in `HardwareDefs.hpp` | Pins passed to `Initialize()` |
| Library initializes SPI bus | **User must initialize SPI2_HOST first** |
| 6 pin parameters (including MOSI/MISO/SCLK) | 3 pin parameters (CS/INT/RST only) |
| Fixed task priorities | Configurable via Initialize() parameters |

## License

Built on DWM1000_ESP32 (DecaWave driver port). See component licenses.

---

**Version**: 2.0  
**Last Updated**: 2025-12-16
