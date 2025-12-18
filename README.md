# DWM1000_ESP32 Library Documentation

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

The **DWM1000_ESP32** library is an ESP32/ESP32-S3 port of the official DecaWave DW1000 driver library. It provides a complete software stack for interfacing ESP32 microcontrollers with the DW1000 UWB (Ultra-Wideband) transceiver chip via SPI.

**Based on**: Official DecaWave DW1000 driver library  
**Ported by**: Project author  
**Platform**: ESP-IDF (Espressif IoT Development Framework)  
**Target MCUs**: ESP32, ESP32-S3 (tested on ESP32-S3)

## Architecture

The library consists of two main layers:

```
Application Code
       ↓
┌──────────────────────────────────────┐
│   DW1000 Driver (decadriver/)        │  ← Official DecaWave code (unchanged)
│   - deca_device_api.h/c              │
│   - deca_regs.h                      │
│   - deca_types.h                     │
└──────────────────────────────────────┘
       ↓
┌──────────────────────────────────────┐
│   Platform Layer (platform/)         │  ← ESP32 port implementation
│   - deca_spi.c/h                     │     (SPI interface)
│   - deca_gpio.c/h                    │     (GPIO & interrupts)
│   - deca_sleep.c                     │     (Delay functions)
│   - deca_debug.c/h                   │     (Debug utilities)
└──────────────────────────────────────┘
       ↓
┌──────────────────────────────────────┐
│   ESP-IDF Framework                  │
│   - SPI Master Driver                │
│   - GPIO Driver                      │
│   - FreeRTOS                         │
└──────────────────────────────────────┘
       ↓
    Hardware
```

## Key Features of the Port

### 1. ESP-IDF SPI Integration

The port leverages ESP-IDF's SPI master driver which provides:

- **Thread-safe SPI access** - Multiple tasks can safely use the SPI bus
- **DMA support** - Efficient data transfers without CPU intervention
- **Hardware CS control workaround** - Manual GPIO control to avoid ESP32-S3 silicon bug
- **Multi-device support** - Share SPI bus with other devices (e.g., IMU)
- **Automatic arbitration** - `spi_device` handle system manages bus access

**Why this matters**: Traditional Arduino SPI libraries don't provide thread-safe multi-device support. ESP-IDF's driver handles all synchronization automatically, making it safe to use DW1000 alongside other SPI devices.

### 2. FreeRTOS Interrupt Handling

Interrupt processing uses FreeRTOS task notifications:

```
DW1000 IRQ Pin → GPIO ISR (IRAM) → Task Notification → ISR Task → dwt_isr() → Callbacks
```

**Benefits**:
- ISR executes in task context (not IRQ context)
- Can call blocking functions safely
- Integrates with FreeRTOS scheduler
- Lower interrupt latency overhead

### 3. Automatic Bus Management

Optional automatic SPI bus acquisition:

```cpp
dwt_readdevid();  // Automatically acquires and releases bus

// Manual mode (for ISR callbacks)
decaIrqStatus_t stat = decamutexon();
dwt_readdevid();
dwt_writetxdata(...);
decamutexoff(stat);
```

## Platform Layer Functions

### SPI Functions (deca_spi.h/c)

#### Initialization

##### dw1000_spi_init()
```cpp
int dw1000_spi_init(
    spi_host_device_t spi_peripheral,
    gpio_num_t io_cs,
    const spi_bus_config_t *spi_bus_cfg
);
```

**Purpose**: Initialize SPI interface for DW1000

**Parameters**:
- `spi_peripheral` - SPI host (e.g., `SPI2_HOST`, `SPI3_HOST`)
- `io_cs` - Chip select GPIO pin number
- `spi_bus` - Pointer to SPI bus configuration, or `NULL` if bus already initialized

**Returns**: 
- `0` on success
- `-1` on error

**Usage**:
```cpp
spi_bus_config_t spi_bus_cfg = {
    .mosi_io_num = SPI_MOSI_PIN,
    .miso_io_num = SPI_MISO_PIN,
    .sclk_io_num = SPI_CLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1024,
};

if (dw1000_spi_init(SPI2_HOST, UWB_CS_PIN, &spi_bus_cfg) != 0) {
    // Handle error
}
```

**Notes**:
- Pass `NULL` for `spi_bus` if sharing with another device that already initialized the bus
- Configures SPI for Mode 0 (CPOL=0, CPHA=0)
- Initial speed: 2 MHz (low speed for initialization)
- After `dwt_initialise()`, call `spi_set_rate_high()` for 16 MHz operation

##### dw1000_spi_deinit()
```cpp
void dw1000_spi_deinit(void);
```

**Purpose**: De-initialize SPI interface and remove device from bus

#### Speed Control

##### spi_set_rate_low()
```cpp
void spi_set_rate_low(void);
```

**Purpose**: Set SPI to 2 MHz for initialization and register access during startup

##### spi_set_rate_high()
```cpp
void spi_set_rate_high(void);
```

**Purpose**: Set SPI to 16 MHz for normal operation

**Usage**:
```cpp
dwt_initialise(DWT_LOADUCODE);
spi_set_rate_high();  // Switch to high speed after init
```

#### Bug Workaround

##### dw1000_spi_fix_bug()
```cpp
void dw1000_spi_fix_bug(void);
```

**Purpose**: Apply SPI workaround for DW1000 silicon bug

**When to call**: After hardware reset and before/after `dwt_initialise()`

**What it does**:
1. Temporarily sets SPI to high speed
2. Sets bit 10 in SYS_CFG register
3. Returns to previous SPI speed

**Usage**:
```cpp
dw1000_hard_reset();
dw1000_spi_fix_bug();  // Must call after reset
dwt_initialise(DWT_LOADUCODE);
spi_set_rate_high();
```

#### Bus Management

Bus management is automatic by default, but you can manually force holding the bus for longer using mutex functions:

```cpp
decaIrqStatus_t stat = decamutexon();  // Lock bus
// Perform multiple SPI operations
decamutexoff(stat);                 // Unlock bus
```

#### Low-Level Functions

These are called by the DW1000 driver - typically not used directly:

##### openspi() / closespi()
```cpp
int openspi(void);
int closespi(void);
```

Abstract SPI open/close (called by driver initialization)

##### writetospi()
```cpp
int writetospi(
    uint16 headerLength,
    const uint8 *headerBuffer,
    uint32 bodylength,
    const uint8 *bodyBuffer
);
```

Write to DW1000 via SPI (separate header and body buffers)

##### readfromspi()
```cpp
int readfromspi(
    uint16 headerLength,
    const uint8 *headerBuffer,
    uint32 readlength,
    uint8 *readBuffer
);
```

Read from DW1000 via SPI

**Returns**: Offset into read buffer where data starts, or `-1` on error

### GPIO Functions (deca_gpio.h/c)

#### Initialization

##### dw1000_gpio_init()
```cpp
int dw1000_gpio_init(
    gpio_num_t io_rst,
    gpio_num_t io_irq,
    gpio_num_t io_wake
);
```

**Purpose**: Initialize GPIO pins for DW1000

**Parameters**:
- `io_rst` - Reset pin (output)
- `io_irq` - Interrupt request pin (input)
- `io_wake` - Wake-up pin (output), use `-1` or `GPIO_NUM_NC` if not used

**Returns**:
- `0` on success
- `-1` on error

**Notes**: Only configures pins - does not setup interrupts (use `dw1000_setup_isr()` for that)

##### dw1000_gpio_deinit()
```cpp
void dw1000_gpio_deinit(void);
```

**Purpose**: De-initialize GPIO and cleanup ISR if configured

#### Reset and Wake

##### dw1000_hard_reset()
```cpp
void dw1000_hard_reset(void);
```

**Purpose**: Hardware reset of DW1000 using RST pin

**Timing**:
1. Assert RST low
2. Hold 1 ms
3. Deassert RST high

⚠️ **Warning**: Must call `dw1000_spi_fix_bug()` after reset

##### dw1000_wake_up()
```cpp
void dw1000_wake_up(void);
```

**Purpose**: Wake DW1000 from sleep using WAKE pin

**Timing**:
1. Assert WAKE low
2. Hold 1 ms
3. Deassert WAKE high

#### Interrupt Setup

##### dw1000_setup_isr()
```cpp
int dw1000_setup_isr(
    uint32_t task_priority,
    dwt_cb_t cbTxDone,
    dwt_cb_t cbRxOk,
    dwt_cb_t cbRxTo,
    dwt_cb_t cbRxErr
);
```

**Purpose**: Setup complete interrupt handling system for DW1000

**Parameters**:
- `task_priority` - FreeRTOS task priority (0 = use default 6)
- `cbTxDone` - TX done callback (NULL if not needed)
- `cbRxOk` - RX success callback (NULL if not needed)
- `cbRxTo` - RX timeout callback (NULL if not needed)
- `cbRxErr` - RX error callback (NULL if not needed)

**Returns**:
- `0` on success
- `-1` on error

**What it does**:
1. Creates FreeRTOS task named "dw1000_isr"
2. Configures GPIO interrupt on IRQ pin (rising edge)
3. Registers ISR handler
4. Configures DW1000 driver callbacks

**Callback signature**:
```cpp
typedef void (*dwt_cb_t)(const dwt_cb_data_t *data);

typedef struct {
    uint32 status;      // Status register value
    uint16 datalength;  // RX data length
    uint8 fctrl[2];     // Frame control
    uint8 rx_flags;     // RX flags
} dwt_cb_data_t;
```

**Example**:
```cpp
void tx_done_cb(const dwt_cb_data_t *cb_data) {
    Serial.println("TX complete");
}

void rx_ok_cb(const dwt_cb_data_t *cb_data) {
    Serial.printf("RX: %d bytes\n", cb_data->datalength);
}

// In setup()
dw1000_setup_isr(6, &tx_done_cb, &rx_ok_cb, NULL, NULL);
dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG, 1);
```

**Task stack size**: 4096 bytes (sufficient for typical callback operations)

##### dw1000_unregister_isr()
```cpp
void dw1000_unregister_isr(void);
```

**Purpose**: Unregister interrupt handling and delete ISR task

#### Interrupt Control

##### dw1000_gpio_enable_irq()
```cpp
void dw1000_gpio_enable_irq(void);
```

**Purpose**: Enable DW1000 IRQ interrupt at GPIO level

##### dw1000_gpio_disable_irq()
```cpp
void dw1000_gpio_disable_irq(void);
```

**Purpose**: Disable DW1000 IRQ interrupt at GPIO level

##### dw1000_gpio_get_irq_status()
```cpp
bool dw1000_gpio_get_irq_status(void);
```

**Purpose**: Get current IRQ enable status

**Returns**: `true` if enabled, `false` if disabled

### Debug Functions (deca_debug.h/c)

#### Status/State Reading

##### deca_get_sys_status()
```cpp
void deca_get_sys_status(sys_status_reg_t *status);
```

**Purpose**: Read and parse SYS_STATUS register

**Structure**:
```cpp
typedef struct {
    uint32_t status_low;   // Octets 0-3
    uint8_t status_high;   // Octet 4
} sys_status_reg_t;
```

##### deca_get_sys_state()
```cpp
void deca_get_sys_state(sys_state_reg_t *state);
```

**Purpose**: Read and parse SYS_STATE register

**Structure**:
```cpp
typedef struct {
    uint8_t tx_state;      // Transmitter state (tx_state_e)
    uint8_t rx_state;      // Receiver state (rx_state_e)
    uint8_t pmsc_state;    // Power management state (pmsc_state_e)
} sys_state_reg_t;
```

**State enumerations**:
```cpp
// Power management states
typedef enum {
    PMSC_STATE_INIT     = 0x0,  // Initializing
    PMSC_STATE_IDLE     = 0x1,  // Idle
    PMSC_STATE_TX_WAIT  = 0x2,  // Waiting to TX
    PMSC_STATE_RX_WAIT  = 0x3,  // Waiting to RX
    PMSC_STATE_TX       = 0x4,  // Transmitting
    PMSC_STATE_RX       = 0x5   // Receiving
} pmsc_state_e;

// Receiver states (partial list)
typedef enum {
    RX_STATE_IDLE           = 0x00,
    RX_STATE_RX_READY       = 0x04,
    RX_STATE_PREAMBLE_FIND  = 0x05,
    RX_STATE_SFD_FOUND      = 0x07,
    RX_STATE_DATA_RX_SEQ    = 0x0C,
    // ... (see deca_debug.h for complete list)
} rx_state_e;

// Transmitter states
typedef enum {
    TX_STATE_IDLE       = 0x0,
    TX_STATE_PREAMBLE   = 0x1,
    TX_STATE_SFD        = 0x2,
    TX_STATE_PHR        = 0x3,
    TX_STATE_DATA       = 0x5,
    // ...
} tx_state_e;
```

#### String Formatting

##### deca_get_status_string()
```cpp
void deca_get_status_string(
    const sys_status_reg_t *status,
    char *buffer,
    size_t buffer_size
);
```

**Purpose**: Convert status register to human-readable string

**Example output**:
```
"TXFRS RXFCG LDEDONE"
```

##### deca_get_state_string()
```cpp
void deca_get_state_string(
    const sys_state_reg_t *state,
    char *buffer,
    size_t buffer_size
);
```

**Purpose**: Convert state register to human-readable string

**Example output**:
```
"PMSC:IDLE TX:IDLE RX:PREAMBLE_FIND"
```

**Usage example**:
```cpp
sys_status_reg_t status;
sys_state_reg_t state;
char status_str[256], state_str[256];

deca_get_sys_status(&status);
deca_get_sys_state(&state);
deca_get_status_string(&status, status_str, sizeof(status_str));
deca_get_state_string(&state, state_str, sizeof(state_str));

Serial.printf("Status: %s\n", status_str);
Serial.printf("State: %s\n", state_str);
```

### Sleep Functions (deca_sleep.c)

Simple delay implementation:

```cpp
void deca_sleep(unsigned int time_ms);  // Millisecond delay
void deca_usleep(unsigned long time_us); // Microsecond delay
```

Uses FreeRTOS `vTaskDelay()` for ms delays and `esp_rom_delay_us()` for µs delays.

### Mutex Functions (deca_spi.c)

FreeRTOS mutex wrappers (if needed by driver):

```cpp
void decamutexon(void);   // Lock mutex
void decamutexoff(void);  // Unlock mutex
```

## Integration with Official Driver

The platform layer implements the hardware abstraction required by the official DW1000 driver:

### Required Platform Functions

| Function | Purpose | Implementation |
|----------|---------|----------------|
| `openspi()` | Initialize SPI | ESP-IDF `spi_bus_add_device()` |
| `closespi()` | Close SPI | ESP-IDF `spi_bus_remove_device()` |
| `writetospi()` | SPI write | ESP-IDF `spi_device_polling_transmit()` |
| `readfromspi()` | SPI read | ESP-IDF `spi_device_polling_transmit()` |
| `deca_sleep()` | Delay | FreeRTOS `vTaskDelay()` |

### DW1000 Driver Functions (unchanged)

The `decadriver/` folder contains the official DecaWave driver code. Key functions include:

**Initialization**:
- `dwt_initialise()` - Initialize device
- `dwt_configure()` - Apply configuration

**TX/RX**:
- `dwt_writetxdata()` / `dwt_readrxdata()` - Frame data
- `dwt_starttx()` / `dwt_rxenable()` - Start operations

**Timing**:
- `dwt_readtxtimestamp()` / `dwt_readrxtimestamp()` - Read timestamps
- `dwt_setdelayedtrxtime()` - Schedule delayed TX/RX

**Configuration**:
- `dwt_setinterrupt()` - Enable interrupts
- `dwt_setrxantennadelay()` / `dwt_settxantennadelay()` - Antenna delays
- `dwt_setrxtimeout()` / `dwt_setrxaftertxdelay()` - Timeouts

See official DecaWave documentation for complete API reference.

## Usage Patterns

### Basic Initialization

```cpp
// 1. Configure SPI bus
spi_bus_config_t spi_bus_cfg = {
    .mosi_io_num = SPI_MOSI_PIN,
    .miso_io_num = SPI_MISO_PIN,
    .sclk_io_num = SPI_CLK_PIN,
    .max_transfer_sz = 1024,
};

// 2. Initialize DW1000 SPI
dw1000_spi_init(SPI2_HOST, UWB_CS_PIN, &spi_bus_cfg);

// 3. Initialize GPIO
dw1000_gpio_init(UWB_RST_PIN, UWB_IRQ_PIN, GPIO_NUM_NC);

// 4. Reset and workaround
dw1000_hard_reset();
dw1000_spi_fix_bug();
delay(5);

// 5. Initialize driver
dwt_initialise(DWT_LOADUCODE);

// 6. High-speed SPI
spi_set_rate_high();

// 7. Configure
dwt_configure(&config);
```

### Polling Mode (Simple)

```cpp
// Transmit
dwt_writetxdata(len, data, 0);
dwt_writetxfctrl(len, 0, 0);
dwt_starttx(DWT_START_TX_IMMEDIATE);
while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS));
dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

// Receive
dwt_rxenable(DWT_START_RX_IMMEDIATE);
while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG));
dwt_readrxdata(buffer, frame_len, 0);
```

### Interrupt Mode

```cpp
// Setup callbacks
void tx_cb(const dwt_cb_data_t *data) {
    Serial.println("TX done");
}

void rx_cb(const dwt_cb_data_t *data) {
    dwt_readrxdata(buffer, data->datalength, 0);
}

// In setup()
dw1000_setup_isr(6, &tx_cb, &rx_cb, NULL, NULL);
dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG, 1);

// In loop()
dwt_starttx(DWT_START_TX_IMMEDIATE);
// Callback handles TX completion automatically
```

### Multi-Device SPI Bus

```cpp
// Share bus with IMU
spi_bus_init(...);  // Initialize bus once

// Add DW1000
dw1000_spi_init(SPI2_HOST, UWB_CS_PIN, NULL);  // NULL = bus already init

// Add IMU
spi_bus_add_device(SPI2_HOST, &imu_cfg, &imu_handle);

// Use devices
decaIrqStatus_t stat = decamutexon();
dwt_readdevid();
decamutexoff(stat);

spi_device_acquire_bus(imu_handle, portMAX_DELAY);
imu_read();
spi_device_release_bus(imu_handle);
```

## Important Notes

### SPI Bus Sharing

✅ **ESP-IDF handles all arbitration automatically**
- Multiple `spi_device_handle_t` on same bus
- Thread-safe by design
- No manual locking needed (unless using manual bus acquisition mode)

### Manual CS Control

Due to ESP32-S3 silicon bug with full-duplex SPI and automatic CS, all chip select operations are done manually via GPIO:

```cpp
gpio_set_level(CS_PIN, 0);  // Assert
spi_device_polling_transmit(...);
gpio_set_level(CS_PIN, 1);  // Deassert
```

This is handled internally by the platform layer.

### Reset Sequence

**Critical sequence**:
```cpp
dw1000_hard_reset();      // 1. Hardware reset
dw1000_spi_fix_bug();     // 2. SPI workaround
dwt_initialise(...);      // 3. Initialize driver
spi_set_rate_high();      // 4. Switch to high speed
```

Order matters! The SPI bug fix must be applied after reset.

### ISR Callback Context

Callbacks execute in a FreeRTOS task (not hardware ISR), so:

✅ Can call blocking functions (with caution)
✅ Can use `Serial.print()`
✅ Can read DW1000 registers
✅ Can use FreeRTOS primitives

⚠️ Keep callbacks short - don't block ISR task too long
⚠️ Use manual bus acquisition if doing multiple SPI operations

### Memory Usage

- **ISR task stack**: 4096 bytes
- **SPI buffers**: 1030 bytes each (TX and RX)
- **Device handle**: ~few hundred bytes
- **Callback overhead**: Minimal

## Example Code References

See the `ESP32 examples/` folder for complete working examples demonstrating all features of this library.

## Credits

**Original DW1000 Driver**: DecaWave Ltd (now part of Qorvo)  
**ESP32 Platform Port**: Project author
**Framework**: Espressif ESP-IDF

## License

Refer to `LICENSE` and `disclaimer.txt` files in the library directory.
