# DW1000Manager Examples

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

This directory contains example applications demonstrating DW1000Manager library usage.

## Important Setup Requirements

### SPI Bus Pre-initialization

**CRITICAL**: Before using any DW1000Manager functions, you **must** initialize the SPI bus (SPI2_HOST):

```cpp
// Step 1: Initialize SPI bus
spi_bus_config_t spi_cfg = {
    .mosi_io_num = SPI_MOSI_PIN,
    .miso_io_num = SPI_MISO_PIN,
    .sclk_io_num = SPI_CLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024,
    .flags = 0,
    .intr_flags = 0
};
spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);

// Step 2: Initialize DW1000 device (only device pins needed)
UWBRanging::Initiator::Initialize(UWB_CS_PIN, UWB_IRQ_PIN, UWB_RST_PIN);
```

### Initialize() Parameters

The `Initialize()` function requires only **device-specific pins**:
- **cs_pin**: SPI chip select for DW1000
- **int_pin**: Interrupt pin from DW1000  
- **rst_pin**: Reset pin for DW1000
- **callback_priority** (optional): ISR task priority (default 6)
- **ranging_priority** (optional, Initiator only): Ranging task priority (default 1)

**Note**: SPI bus pins (MOSI, MISO, SCLK) are NOT needed since the bus is already initialized.

## Available Examples

### 1. Initiator (TestInitiator.cpp)

**Location**: `/src/TestInitiator.cpp`

**Purpose**: Demonstrates automatic ranging at 20ms intervals (50 Hz)

**Features**:
- Initializes UWB as initiator
- Starts automatic ranging task
- Monitors and prints status every 5 seconds
- Demonstrates `IsActive()` status checking

**Code**:
```cpp
#include <Arduino.h>
#include <DW1000Initiator.hpp>
#include <HardwareDefs.hpp>
#include <driver/spi_master.h>

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== UWB DS-TWR Initiator Test ===");
    Serial.println("Initializing...");
    
    // Step 1: Initialize SPI bus
    spi_bus_config_t spi_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0
    };
    
    if (spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        Serial.println("ERROR: Failed to initialize SPI bus!");
        while (1) { delay(1000); }
    }
    
    // Step 2: Initialize UWB hardware (only device pins needed)
    if (!UWBRanging::Initiator::Initialize(UWB_CS_PIN, UWB_IRQ_PIN, UWB_RST_PIN)) {
        Serial.println("ERROR: Failed to initialize UWB Initiator!");
        while (1) { delay(1000); }
    }
    
    Serial.println("UWB Initiator initialized successfully");
    
    // Start ranging with 20ms interval (50 Hz)
    UWBRanging::Initiator::Begin(20);
    
    Serial.println("Ranging started with 20ms interval");
    Serial.println("Initiator is now sending ranging polls...\n");
}

void loop() {
    // Check if still active
    if (UWBRanging::Initiator::IsActive()) {
        static uint32_t last_status_time = 0;
        uint32_t current_time = millis();
        
        // Print status every 5 seconds
        if (current_time - last_status_time >= 5000) {
            Serial.println("Initiator active and ranging...");
            last_status_time = current_time;
        }
    } else {
        Serial.println("WARNING: Initiator is not active!");
    }
    
    delay(100);
}
```

**Expected Behavior**:
1. Prints initialization message
2. Starts ranging at 50 Hz
3. Prints "Initiator active and ranging..." every 5 seconds
4. Ranging runs continuously in background task

**Troubleshooting**:
- If initialization fails, check hardware connections
- Enable DEBUG_INIT for detailed error messages
- Verify 3.3V power supply

---

### 2. Responder (TestResponder.cpp)

**Location**: `/src/TestResponder.cpp`

**Purpose**: Demonstrates distance measurement with real-time statistics

**Features**:
- Initializes UWB as responder
- Receives ranging results from internal queue
- Calculates per-second statistics:
  - Update rate (Hz)
  - Mean distance (m)
  - Standard deviation (cm)
  - Max difference - range spread (cm)
- Demonstrates queue handling
- Formatted output with units

**Code**:
```cpp
#include <Arduino.h>
#include <DW1000Responder.hpp>
#include <HardwareDefs.hpp>
#include <driver/spi_master.h>
#include <vector>
#include <algorithm>
#include <cmath>

QueueHandle_t ranging_queue = NULL;

struct Statistics {
    float mean;
    float std_dev;
    float max_diff;
    uint32_t count;
    float update_rate_hz;
};

Statistics CalculateStatistics(const std::vector<double>& distances, uint32_t time_window_ms) {
    Statistics stats = {0, 0, 0, 0, 0};
    
    if (distances.empty()) {
        return stats;
    }
    
    stats.count = distances.size();
    stats.update_rate_hz = (stats.count * 1000.0f) / time_window_ms;
    
    // Calculate mean
    double sum = 0;
    for (double d : distances) {
        sum += d;
    }
    stats.mean = sum / stats.count;
    
    // Calculate standard deviation
    double variance_sum = 0;
    for (double d : distances) {
        double diff = d - stats.mean;
        variance_sum += diff * diff;
    }
    stats.std_dev = sqrt(variance_sum / stats.count);
    
    // Calculate max difference (max - min)
    double min_val = *std::min_element(distances.begin(), distances.end());
    double max_val = *std::max_element(distances.begin(), distances.end());
    stats.max_diff = max_val - min_val;
    
    return stats;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== UWB DS-TWR Responder Test ===");
    Serial.println("Initializing...");
    
    // Step 1: Initialize SPI bus
    spi_bus_config_t spi_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0
    };
    
    if (spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        Serial.println("ERROR: Failed to initialize SPI bus!");
        while (1) { delay(1000); }
    }
    
    // Step 2: Initialize UWB hardware (returns queue handle)
    ranging_queue = UWBRanging::Responder::Initialize(UWB_CS_PIN, UWB_IRQ_PIN, UWB_RST_PIN);
    
    if (ranging_queue == NULL) {
        Serial.println("ERROR: Failed to initialize UWB Responder!");
        while (1) { delay(1000); }
    }
    
    Serial.println("UWB Responder initialized successfully");
    
    // Start receiving
    UWBRanging::Responder::Begin();
    
    Serial.println("Responder started and listening...");
    Serial.println("\nFormat: UWB stats | Frequency: XX Hz | Mean: XXX.XXX m | Std: XX.X cm | MaxDiff: XX.X cm");
    Serial.println("------------------------------------------------------------------------------------------\n");
}

void loop() {
    static std::vector<double> distances;
    static uint32_t last_print_time = millis();
    static uint32_t window_start_time = millis();
    
    uint32_t current_time = millis();
    
    // Receive all available ranging results
    UWBRanging::RangingResult result;
    while (xQueueReceive(ranging_queue, &result, 0) == pdTRUE) {
        distances.push_back(result.distance_m);
    }
    
    // Print statistics every second
    if (current_time - last_print_time >= 1000) {
        uint32_t time_window = current_time - window_start_time;
        
        if (!distances.empty()) {
            // Calculate and print statistics
            Statistics stats = CalculateStatistics(distances, time_window);
            
            Serial.printf("UWB stats | Frequency: %.0f Hz | Mean: %.3f m | Std: %.1f cm | MaxDiff: %.1f cm\n",
                          stats.update_rate_hz,
                          stats.mean,
                          stats.std_dev * 100.0f,  // Convert m to cm
                          stats.max_diff * 100.0f); // Convert m to cm
            
            // Clear distances for next window
            distances.clear();
            window_start_time = current_time;
        } else {
            Serial.println("UWB stats | Frequency: 0 Hz | Mean: --- m | Std: --- cm | MaxDiff: --- cm");
        }
        
        last_print_time = current_time;
    }
    
    // Check if still active
    if (!UWBRanging::Responder::IsActive()) {
        Serial.println("WARNING: Responder is not active!");
    }
    
    delay(10);
}
```

**Expected Output**:
```
=== UWB DS-TWR Responder Test ===
Initializing...
UWB Responder initialized successfully
Responder started and listening...

Format: UWB stats | Frequency: XX Hz | Mean: XXX.XXX m | Std: XX.X cm | MaxDiff: XX.X cm
------------------------------------------------------------------------------------------

UWB stats | Frequency: 48 Hz | Mean: 1.234 m | Std: 2.3 cm | MaxDiff: 8.5 cm
UWB stats | Frequency: 51 Hz | Mean: 1.235 m | Std: 2.1 cm | MaxDiff: 7.8 cm
UWB stats | Frequency: 49 Hz | Mean: 1.233 m | Std: 2.4 cm | MaxDiff: 9.1 cm
```

**Statistics Explained**:
- **Frequency**: Number of measurements received per second
- **Mean**: Average distance over the last second
- **Std**: Standard deviation showing measurement consistency
- **MaxDiff**: Range (max - min), indicating worst-case spread

**Troubleshooting**:
- If frequency is 0, check initiator is running and in range
- High std dev may indicate multipath or metal objects nearby
- Low frequency may indicate SPI bus issues or range problems

---

## Running the Examples

### Using PlatformIO

1. **For Initiator**:
   ```bash
   # Copy TestInitiator.cpp to src/main.cpp
   cp src/TestInitiator.cpp src/main.cpp
   pio run --target upload
   pio device monitor
   ```

2. **For Responder**:
   ```bash
   # Copy TestResponder.cpp to src/main.cpp
   cp src/TestResponder.cpp src/main.cpp
   pio run --target upload
   pio device monitor
   ```

### Using Arduino IDE

1. Open example file in Arduino IDE
2. Select board: ESP32-S3 Dev Module
3. Configure board settings:
   - CPU Frequency: 240MHz
   - Flash Size: 8MB
   - PSRAM: 2MB
4. Upload and open Serial Monitor at 115200 baud

## Testing Setup

### Minimal Test Setup

1. **Hardware**: 2x ESP32-S3 + 2x DW1000 modules
2. **Wiring**: Follow hardware setup in main README
3. **Distance**: Start at < 1 meter for initial testing
4. **Environment**: Open area, away from metal objects

### Testing Procedure

1. **Upload Initiator** to first device
2. **Upload Responder** to second device
3. **Power both devices**
4. **Monitor responder** serial output
5. **Expect**: ~50 Hz update rate, stable distance readings
6. **Vary distance**: Move devices to test range
7. **Check accuracy**: Compare with tape measure

### Expected Performance

At 1-2 meters distance in good conditions:
- **Update Rate**: 45-52 Hz (target: 50 Hz)
- **Mean**: Within ±5 cm of actual distance
- **Std Dev**: 2-4 cm typical
- **MaxDiff**: 5-12 cm typical

### Common Test Issues

**No measurements received**:
- Verify both devices powered and initialized
- Check serial output for errors
- Reduce distance to < 50 cm initially
- Enable DEBUG_RANGING in both

**Low update rate (<30 Hz)**:
- Check SPI bus speed (should be 16 MHz after init)
- Verify no SPI conflicts with other devices
- Check FreeRTOS task priorities

**High variance (std dev >10 cm)**:
- Move to open area
- Remove metal objects nearby
- Check antennas not blocked
- Consider antenna delay calibration

**Inconsistent results**:
- Ensure devices stationary
- Check power supply stability
- Verify 3.3V regulation
- Test in different location

## Advanced Usage

### Custom Statistics

Modify responder example to add:
- Median calculation
- Running average filter
- Outlier rejection
- Rate limiting

### Multi-Device Setup

For tracking multiple initiators:
- Modify message headers to include ID
- Add routing logic in responder
- Maintain separate statistics per device

### Data Logging

Add SD card logging:
```cpp
// In loop, after receiving result
File logFile = SD.open("/ranging.csv", FILE_APPEND);
logFile.printf("%llu,%.3f\n", result.measurement_time_us, result.distance_m);
logFile.close();
```

### Visualization

Send data to Python script for real-time plotting:
```python
import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyUSB0', 115200)
distances = []

while True:
    line = ser.readline().decode()
    if "Mean:" in line:
        dist = float(line.split("Mean:")[1].split("m")[0])
        distances.append(dist)
        plt.plot(distances)
        plt.pause(0.01)
```

## Summary

Both examples demonstrate the simplified initialization process:

1. **Initialize SPI bus** (SPI2_HOST) with standard ESP32 SPI configuration
2. **Call Initialize()** with only 3 device pins (CS, INT, RST)  
3. **Call Begin()** to start operation
4. **Use results** from queue (Responder) or rely on automatic polling (Initiator)

The key change from previous versions is that **SPI bus initialization is now the user's responsibility**, allowing better integration with multi-device SPI systems. The library only configures the DW1000 device itself.

## Implementation Notes

### Updated Timing Control (v2.1)

The library has been updated to match the official DS-TWR examples for improved reliability:

**Initiator:**
- Timing parameters (`dwt_setrxaftertxdelay`, `dwt_setrxtimeout`) are now set **before** sending POLL in `TriggerRanging()`
- TX confirmation callback simplified: no `dwt_forcetrxoff`, just enables RX
- Single `dwt_forcetrxoff` at the start of each ranging exchange

**Responder:**
- Timing parameters now set in **RxOkCallback** before sending RESPONSE (not in TX callback)
- TX confirmation callback simplified: no `dwt_forcetrxoff`, just enables RX
- Continuous RX mode maintained throughout operation

**Benefits:**
- Reduced state transitions
- Better timing synchronization
- Improved ranging success rate
- Matches reference implementation behavior

For complete API documentation, see [../README.md](../README.md).

---

**Last Updated**: 2025-12-17  
**Version**: 2.1 - Updated timing control to match official DS-TWR examples
