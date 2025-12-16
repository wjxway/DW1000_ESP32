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

void setup() {
    // Initialize hardware
    if (!UWBRanging::Initiator::Initialize()) {
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

QueueHandle_t queue;

void setup() {
    // Initialize and get queue handle
    queue = UWBRanging::Responder::Initialize();
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
| `bool Initialize(uint32_t callback_priority = 6, uint32_t ranging_priority = 1)` | Initialize UWB hardware, optionally specify task priorities |
| `void Begin(uint32_t interval_ms)` | Start ranging (0 = manual mode) |
| `void Stop()` | Stop ranging |
| `bool TriggerRanging()` | Manually trigger one ranging exchange |
| `bool IsActive()` | Check if initialized and running |

**Task Priorities:** ISR task handles hardware interrupts; ranging task sends poll messages.

### Responder

| Function | Description |
|----------|-------------|
| `QueueHandle_t Initialize(uint32_t callback_priority = 6)` | Initialize UWB hardware, returns queue handle, optionally specify ISR task priority |
| `void Begin()` | Start receiving ranging requests |
| `void Stop()` | Stop receiving |
| `bool IsActive()` | Check if initialized and running |

## Hardware Setup

Pin configuration (defined in `HardwareDefs.hpp`):

```
ESP32-S3          DW1000
--------          ------
GPIO 11  -------> MOSI
GPIO 13  <------- MISO
GPIO 12  -------> SCLK
GPIO 10  -------> CS
GPIO 14  -------> RST
GPIO 9   <------- IRQ
3.3V     -------> VDD
GND      -------> GND
```

**Important**: DW1000 is 3.3V only!

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

## Migration from Old API

| Old | New |
|-----|-----|
| `begin()` | `Initialize()` |
| `triggerRanging()` | `TriggerRanging()` |
| `isActive()` | `IsActive()` |
| `setInterval(ms)` | `Begin(ms)` |
| External queue creation | Internal (returned by `Initialize()`) |

## License

Built on DWM1000_ESP32 (DecaWave driver port). See component licenses.

---

**Version**: 1.0  
**Last Updated**: 2025-12-16
    
    // Create queue for ranging results
    ranging_queue = xQueueCreate(10, sizeof(UWBRanging::RangingResult));
    
    if (ranging_queue == NULL) {
        Serial.println("Failed to create queue");
        while(1);
    }
    
    // Initialize UWB as responder with queue
    if (!UWBRanging::Responder::begin(ranging_queue)) {
        Serial.println("Failed to initialize UWB Responder");
        while(1);
    }
    
    Serial.println("UWB Responder ready");
}

void loop() {
    UWBRanging::RangingResult result;
    
    // Check for new ranging results
    if (xQueueReceive(ranging_queue, &result, 0) == pdTRUE) {
        Serial.printf("Distance: %.3f m, Time: %llu us\n", 
                      result.distance_m, 
                      result.measurement_time_us);
    }
    
    delay(10);
}
```

## API Reference

### Data Structures

#### `UWBRanging::RangingResult`
- `double distance_m` - Distance in meters
- `uint64_t measurement_time_us` - Timestamp in microseconds (from `esp_timer_get_time()`)

### Initiator Functions

#### `bool UWBRanging::Initiator::begin()`
Initialize UWB hardware as initiator. Returns `true` on success.

#### `bool UWBRanging::Initiator::triggerRanging()`
Trigger a single ranging exchange (non-blocking). Returns `true` if successfully initiated.
Note: Initiator only triggers the exchange; distance is calculated by the responder.

#### `bool UWBRanging::Initiator::isActive()`
Check if initiator is initialized and ready. Returns `true` if active.

### Responder Functions

#### `bool UWBRanging::Responder::begin(QueueHandle_t result_queue)`
Initialize UWB hardware as responder. 
- `result_queue` - FreeRTOS queue handle for receiving results (must be created with `sizeof(RangingResult)`)
- Returns `true` on success

When ranging completes, results are automatically pushed to the queue from ISR context.

## Configuration

The library uses a fixed configuration optimized for DS-TWR:
- Channel: 1
- PRF: 64 MHz
- Preamble: 128 symbols
- Data rate: 850 kbps
- TX/RX antenna delay: 16436

## Debug Output

To enable debug output, edit `DW1000Manager.cpp`:
```cpp
#define DEBUG_INIT 1     // Initialization messages
#define DEBUG_RANGING 1  // Ranging event messages
```

## Dependencies

- DWM1000_ESP32 library (included in project)
- HardwareDefs library (for pin definitions)
- FreeRTOS (ESP-IDF)
