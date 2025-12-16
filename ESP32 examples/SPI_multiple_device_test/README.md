# SPI_multiple_device_test - ESP-IDF SPI Bus Sharing Test

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

The `SPI_multiple_device_test` folder contains a test program that demonstrates ESP-IDF's SPI bus arbitration mechanism. This example shows how multiple SPI devices can safely share a single SPI bus using ESP-IDF's built-in locking and device management.

## File

**SPI_multiple_device_test.cpp** - Demonstrates multi-device SPI bus sharing with concurrent access from multiple FreeRTOS tasks

## Purpose

- Demonstrate ESP-IDF SPI bus arbitration
- Test concurrent SPI access from multiple tasks
- Validate proper bus acquisition and release
- Show correct chip select handling for multiple devices
- Illustrate FreeRTOS task synchronization with SPI operations

## Concept: SPI Bus Sharing in ESP-IDF

ESP-IDF's `spi_master` driver provides built-in arbitration for multiple devices on the same SPI bus:

```
ESP32-S3 SPI2 Bus
├── MOSI, MISO, CLK (shared)
├── Device 1 (e.g., IMU) - CS1
│   └── spi_device_handle_t spi_dev_1
└── Device 2 (e.g., UWB) - CS2
    └── spi_device_handle_t spi_dev_2
```

### Key Features

1. **Automatic arbitration** - ESP-IDF handles access conflicts
2. **Device handles** - Each device has separate configuration
3. **Bus locking** - `spi_device_acquire_bus()` / `spi_device_release_bus()`
4. **Thread-safe** - Safe for multi-task access
5. **Transaction queuing** - Queued transactions wait for bus availability

## Hardware Simulation

This example simulates two SPI devices on the same bus:
- `spi_dev_1` - Simulates IMU device (with CS on IMU_CS_PIN)
- `spi_dev_2` - Simulates UWB device (with CS on UWB_CS_PIN)

**Note**: Actual hardware devices not required - this is a bus arbitration test.

## Code Structure

### Global Variables

```cpp
spi_device_handle_t spi_dev_1, spi_dev_2;  // Device handles
spi_transaction_t spi_transaction;          // Shared transaction structure
uint8_t tx_buffer_1[100], tx_buffer_2[100]; // Dummy data buffers
```

### FreeRTOS Task

```cpp
void try_spi_task(void *pvParameters)
{
    while (1) {
        // Acquire bus for device 2
        spi_device_acquire_bus(spi_dev_2, portMAX_DELAY);
        
        // Perform SPI transaction
        gpio_set_level(UWB_CS_PIN, 0);                    // Assert CS
        spi_device_polling_transmit(spi_dev_2, &spi_transaction);
        gpio_set_level(UWB_CS_PIN, 1);                    // Deassert CS
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Release bus
        spi_device_release_bus(spi_dev_2);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Main Loop

```cpp
void loop()
{
    // Release bus (from previous acquisition)
    spi_device_release_bus(spi_dev_1);
    
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Acquire bus again
    spi_device_acquire_bus(spi_dev_1, portMAX_DELAY);
    
    vTaskDelay(pdMS_TO_TICKS(1000));
}
```

## Setup Phase

1. **Initialize Serial** - 115200 baud communication
2. **Blink LED** - Visual startup indicator
3. **Configure SPI Device**:
   ```cpp
   spi_device_interface_config_t spi_dev_config;
   spi_dev_config.mode = 0x3;              // CPOL=1, CPHA=1
   spi_dev_config.clock_speed_hz = 1000000; // 1 MHz
   spi_dev_config.spics_io_num = -1;       // Manual CS control
   spi_dev_config.queue_size = 5;          // 5 queued transactions max
   ```

4. **Configure SPI Bus**:
   ```cpp
   spi_bus_config_t bus_config;
   bus_config.mosi_io_num = SPI_MOSI_PIN;
   bus_config.miso_io_num = SPI_MISO_PIN;
   bus_config.sclk_io_num = SPI_CLK_PIN;
   ```

5. **Initialize Bus** (if not already initialized):
   ```cpp
   esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
   // ESP_ERR_INVALID_STATE is OK if bus already initialized
   ```

6. **Add Devices to Bus**:
   ```cpp
   spi_bus_add_device(SPI2_HOST, &spi_dev_config, &spi_dev_1);
   spi_bus_add_device(SPI2_HOST, &spi_dev_config, &spi_dev_2);
   ```

7. **Test Transmission on Device 1**:
   ```cpp
   spi_device_acquire_bus(spi_dev_1, portMAX_DELAY);
   gpio_set_level(IMU_CS_PIN, 0);
   spi_device_polling_transmit(spi_dev_1, &spi_transaction);
   gpio_set_level(IMU_CS_PIN, 1);
   spi_device_release_bus(spi_dev_1);
   ```

8. **Create Concurrent Task**:
   ```cpp
   xTaskCreate(&try_spi_task, "try_spi_task", 4096, NULL, 4, nullptr);
   ```

## Operation Flow

### Timeline

```
Time    Main Loop (spi_dev_1)              Task (spi_dev_2)
  0     [Acquires bus]                     [Blocked - waiting]
100     [Holds bus]                        [Blocked - waiting]
1000    [Releases bus]                     [Acquires bus]
1005    [Waits 5ms]                        [Transmits]
1010    [Tries to acquire]                 [Holds bus]
        [Blocked - waiting]                
1100                                       [Releases bus]
1100    [Acquires bus immediately]         [Waits 100ms]
2100    [Releases bus]                     [Acquires bus]
2105    [Waits 5ms]                        [Transmits]
...
```

### Arbitration Demonstration

1. **Main loop holds spi_dev_1** → Task tries to acquire spi_dev_2 → **Task blocks**
2. **Main loop releases spi_dev_1** → Task acquires spi_dev_2 → **Task proceeds**
3. **Task holds spi_dev_2** → Main loop tries to acquire spi_dev_1 → **Main blocks**
4. **Task releases spi_dev_2** → Main loop acquires spi_dev_1 → **Main proceeds**

## Key ESP-IDF Functions

### Bus Management

#### spi_bus_initialize()
```cpp
esp_err_t spi_bus_initialize(
    spi_host_device_t host,
    const spi_bus_config_t *bus_config,
    spi_dma_chan_t dma_chan
);
```
- Initializes SPI bus (shared by all devices)
- Returns `ESP_OK` on success
- Returns `ESP_ERR_INVALID_STATE` if already initialized (safe to ignore)
- Uses DMA channel for efficient transfers

### Device Management

#### spi_bus_add_device()
```cpp
esp_err_t spi_bus_add_device(
    spi_host_device_t host,
    const spi_device_interface_config_t *dev_config,
    spi_device_handle_t *handle
);
```
- Adds device to SPI bus
- Returns device handle for subsequent operations
- Each device can have different speed, mode, CS pin

#### spi_bus_remove_device()
```cpp
esp_err_t spi_bus_remove_device(spi_device_handle_t handle);
```
- Removes device from bus
- Must not have active transactions

### Bus Locking

#### spi_device_acquire_bus()
```cpp
esp_err_t spi_device_acquire_bus(
    spi_device_handle_t device,
    TickType_t wait
);
```
- **Locks bus exclusively** for this device
- Blocks if another device has lock
- `wait`: `portMAX_DELAY` = wait forever, `0` = try once
- Returns `ESP_OK` when lock acquired
- **Must be released** with `spi_device_release_bus()`

#### spi_device_release_bus()
```cpp
void spi_device_release_bus(spi_device_handle_t device);
```
- Releases bus lock
- Allows other devices to acquire
- **Must match each acquire** - no double-release!

### Transmission

#### spi_device_polling_transmit()
```cpp
esp_err_t spi_device_polling_transmit(
    spi_device_handle_t device,
    spi_transaction_t *trans_desc
);
```
- Performs SPI transaction
- Blocks until complete (polling mode)
- Alternative: `spi_device_transmit()` (DMA, non-blocking)

## Important Notes

### Manual CS Control

```cpp
spi_dev_config.spics_io_num = -1;  // Disable automatic CS
```

**Why?** ESP32-S3 has a hardware bug with automatic CS in full-duplex mode. DW1000 library uses manual GPIO control:

```cpp
gpio_set_level(CS_PIN, 0);  // Assert before transaction
spi_device_polling_transmit(...);
gpio_set_level(CS_PIN, 1);  // Deassert after transaction
```

### Acquire/Release Rules

**✅ Correct:**
```cpp
spi_device_acquire_bus(dev);
// ... multiple transactions ...
spi_device_release_bus(dev);
```

**❌ Incorrect:**
```cpp
spi_device_acquire_bus(dev);
spi_device_acquire_bus(dev);  // Double-acquire → deadlock!
```

**❌ Incorrect:**
```cpp
spi_device_release_bus(dev);
spi_device_release_bus(dev);  // Double-release → assert failure!
```

### When to Use Bus Locking

**Use acquire/release when:**
- Performing multiple related transactions
- Need atomic sequence of operations
- Want guaranteed exclusive access

**Don't use when:**
- Single transaction (use `spi_device_transmit()` directly)
- Don't need exclusivity

## Expected Serial Output

```
[Blink LED]

initial transmission on spi_dev_1 successful!
spi_dev_1 acquired spi bus

[Task starts, blocked by spi_dev_1 holding bus]

loop releasing spi_dev_1
try_spi_task acquired spi_dev_2

[Task transmits on spi_dev_2]

loop acquired spi_dev_1

[Main holds spi_dev_1, task waits]

loop releasing spi_dev_1
try_spi_task releasing spi_dev_2
loop acquired spi_dev_1

[Pattern repeats...]
```

## Demonstrated Behaviors

### 1. Bus Arbitration
- Only one device can hold bus at a time
- Other devices block until bus available

### 2. Task Blocking
- `portMAX_DELAY` causes indefinite wait
- Task yields CPU while waiting

### 3. FreeRTOS Integration
- SPI operations integrate with FreeRTOS scheduler
- Blocked tasks don't consume CPU

### 4. Safe Multi-Threading
- No race conditions
- No data corruption
- Deterministic behavior

## Commented-Out Tests

### Double Acquisition Test (Line 34-35)
```cpp
// spi_device_acquire_bus(spi_dev_2, portMAX_DELAY);
```
**Result**: Would cause indefinite block (task already owns bus)

### Double Release Test (Line 37-38)
```cpp
// spi_device_release_bus(spi_dev_2);
```
**Result**: Would trigger ESP-IDF assertion failure

**Conclusion**: These demonstrate what NOT to do.

## Relevance to DWM1000_ESP32 Library

This test validates the SPI arbitration mechanism used by the DWM1000 library:

### In DWM1000 Library

```cpp
// From deca_spi.c
esp_err_t dw1000_spi_acquire_bus() {
    return spi_device_acquire_bus(dw1000_spi_handle, portMAX_DELAY);
}

void dw1000_spi_release_bus() {
    spi_device_release_bus(dw1000_spi_handle);
}
```

### In Application Code

When sharing SPI bus with IMU:
```cpp
// DWM1000 operations
dw1000_spi_acquire_bus();
dwt_readdevid();
dw1000_spi_release_bus();

// IMU operations
imu_spi_acquire_bus();
imu_read_data();
imu_spi_release_bus();
```

**No conflicts!** ESP-IDF handles arbitration automatically.

## Troubleshooting

| Issue | Symptom | Cause | Solution |
|-------|---------|-------|----------|
| Task never runs | Only main loop output | Task priority too low | Increase task priority |
| Deadlock | Program freezes | Double-acquire | Check acquire/release pairing |
| Assert failure | Crash on release | Double-release | Remove duplicate release |
| Garbled data | Corrupted transmissions | No bus locking | Use acquire/release for sequences |

## Performance Considerations

### Bus Locking Overhead

- **Acquire**: ~10-20 µs (FreeRTOS mutex)
- **Release**: ~10-20 µs
- **Transaction**: Depends on data size and speed

### When to Lock

**Lock for entire sequence:**
```cpp
acquire_bus();
transaction_1();  // 100 µs
transaction_2();  // 100 µs
transaction_3();  // 100 µs
release_bus();
// Total: 300 µs + 30 µs overhead = 330 µs
```

**Lock per transaction:**
```cpp
acquire_bus();
transaction_1();
release_bus();

acquire_bus();
transaction_2();
release_bus();

acquire_bus();
transaction_3();
release_bus();
// Total: 300 µs + 90 µs overhead = 390 µs
```

**Recommendation**: Lock once for related operations.

## Related Concepts

- **Mutex** - FreeRTOS mutual exclusion (used internally by bus locking)
- **DMA** - Direct Memory Access (used by ESP-IDF SPI driver)
- **Full-duplex** - Simultaneous TX/RX (requires manual CS on ESP32-S3)
- **Transaction queuing** - ESP-IDF queues transactions when bus busy

## Use Cases

- **IMU + UWB** - Share SPI bus between sensors
- **Multiple sensors** - Several devices on one bus
- **Task isolation** - Different tasks access different devices safely
- **Dynamic device switching** - Runtime device selection

## Notes

- This is a **test/validation** program, not a production example
- Real applications should use actual device drivers (IMU, UWB)
- ESP-IDF SPI driver is thread-safe by design
- No application-level locking needed when using bus acquire/release
- The DWM1000_ESP32 library leverages this mechanism for safe multi-device operation
- Bus arbitration is handled in hardware and driver - transparent to application
- Always pair acquire/release calls - consider RAII patterns in C++

## References

- ESP-IDF SPI Master Driver documentation
- FreeRTOS task synchronization
- ESP32-S3 Technical Reference Manual (SPI chapter)
- DWM1000_ESP32 library implementation (deca_spi.c)
