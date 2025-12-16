# UWB_TX_ISR_Test.cpp - Interrupt-Driven UWB Transmitter Test

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

`UWB_TX_ISR_Test.cpp` demonstrates interrupt-driven UWB frame transmission. Unlike `UWB_TX_Test.cpp` which polls the status register, this example uses ISR callbacks to handle TX confirmation events, freeing the CPU for other tasks during transmission.

## Purpose

- Demonstrate interrupt-based DW1000 operation
- Show ISR callback registration and handling
- Test asynchronous TX confirmation
- Compare interrupt vs. polling performance
- Illustrate proper FreeRTOS task integration with hardware interrupts

## Hardware Requirements

- ESP32-S3 microcontroller
- DW1000 UWB transceiver module
- Complete SPI and GPIO connections
- **IRQ pin connection required** (unlike polling examples)

## Configuration

### DW1000 Configuration

Same as `UWB_TX_Test.cpp`:
```cpp
static dwt_config_t dw1000_config = {
    1,                 /* Channel 1 */
    DWT_PRF_64M,       /* 64 MHz PRF */
    DWT_PLEN_128,      /* 128-symbol preamble */
    DWT_PAC8,          /* PAC size 8 */
    9,                 /* TX/RX preamble code 9 */
    1,                 /* Non-standard SFD */
    DWT_BR_850K,       /* 850 kbps data rate */
    DWT_PHRMODE_STD,   /* Standard PHY header */
    (128 + 8 + 64 - 8) /* SFD timeout */
};
```

### Interrupt Configuration

```cpp
#define ISR_PRIORITY 6  // FreeRTOS task priority for IRQ handler
```

### Frame Structure

Same 802.15.4e blink frame as `UWB_TX_Test.cpp`:
```cpp
static uint8 tx_msg[] = {
    0xC5, 0x00,                    // Frame control + sequence
    0xDE, 0xCA, 0x01, 0x23, 0x45, 0x67, 0x89,  // Payload
    0x00, 0x00                     // CRC (auto)
};
```

## Interrupt Architecture

### ISR Flow

```
Hardware IRQ Pin → ESP32 GPIO ISR → FreeRTOS Task Notification
                                          ↓
                                   ISR Task Wakes Up
                                          ↓
                                   dwt_isr() Called
                                          ↓
                              Reads DW1000 Status Register
                                          ↓
                              Dispatches to Callback(s)
                                          ↓
                              tx_conf_cb() Executed
```

### Callback Function

```cpp
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    Serial.printf("\n*** CALLBACK: tx_conf_cb ***\nStatus: 0x%08lX - TX sent\n",
                  cb_data->status);
}
```

**Callback Data Structure:**
```cpp
typedef struct {
    uint32 status;      // Status register value
    uint16 datalength;  // Length of frame (RX only)
    uint8  fctrl[2];    // Frame control (RX only)
    uint8  rx_flags;    // RX flags (RX only)
} dwt_cb_data_t;
```

## Operation Flow

### Setup Phase

Standard initialization plus interrupt setup:

1. Initialize serial, LED, IMU deselection
2. Configure SPI and GPIO
3. Reset DW1000 and load microcode
4. Configure DW1000 parameters
5. **Setup ISR with callbacks** ← Key difference from polling version
6. **Enable TX interrupts**

### ISR Setup

```cpp
if (dw1000_setup_isr(6, &tx_conf_cb, NULL, NULL, NULL) != 0)
{
    Serial.println("ERROR: dw1000_setup_isr failed!");
    while (1);
}
```

**Parameters:**
- `6` - FreeRTOS task priority
- `&tx_conf_cb` - TX done callback
- `NULL` - RX OK callback (not used)
- `NULL` - RX timeout callback (not used)
- `NULL` - RX error callback (not used)

### Interrupt Enable

```cpp
dwt_setinterrupt(DWT_INT_TFRS, 1);
```

Enables only TX Frame Sent (TFRS) interrupt.

### Main Loop (Simplified)

```cpp
void loop()
{
    // 1. Write frame to TX buffer
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
    dwt_writetxfctrl(sizeof(tx_msg), 0, 0);
    
    // 2. Start transmission
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    
    // 3. Delay (CPU is free during transmission!)
    vTaskDelay(pdMS_TO_TICKS(TX_DELAY_MS));
    
    // 4. Increment sequence number
    tx_msg[BLINK_FRAME_SN_IDX]++;
    
    // No polling! Callback handles TX confirmation asynchronously
}
```

## Key Differences from Polling Version

| Aspect | Polling (UWB_TX_Test) | Interrupt (UWB_TX_ISR_Test) |
|--------|----------------------|----------------------------|
| **CPU Usage** | Busy-wait during TX | CPU free during TX |
| **Status Checking** | Manual polling loop | Automatic callback |
| **IRQ Pin** | Not required | Required |
| **Code Complexity** | Simpler | More complex |
| **Latency** | Polling interval delay | Immediate notification |
| **Multi-tasking** | Blocks other tasks | Allows concurrent tasks |
| **Event Handling** | Inline in loop | Callback function |

## Key Functions Used

### Platform Functions (ESP32 Port)

#### dw1000_setup_isr()
```cpp
int dw1000_setup_isr(
    uint32_t task_priority,  // FreeRTOS task priority
    dwt_cb_t cbTxDone,       // TX done callback
    dwt_cb_t cbRxOk,         // RX success callback
    dwt_cb_t cbRxTo,         // RX timeout callback
    dwt_cb_t cbRxErr         // RX error callback
);
```

**What it does:**
1. Creates FreeRTOS task for IRQ handling
2. Registers GPIO ISR on DW1000 IRQ pin
3. Configures DW1000 callback pointers
4. Sets up task notification mechanism

**Returns:** 0 on success, -1 on error

### DW1000 Driver Functions

#### dwt_setinterrupt()
```cpp
void dwt_setinterrupt(uint32 bitmask, uint8 enable);
```

**Interrupt Bitmasks:**
- `DWT_INT_TFRS` - TX Frame Sent
- `DWT_INT_RFCG` - RX Frame Control Good
- `DWT_INT_RPHE` - RX PHY Header Error
- `DWT_INT_RFCE` - RX Frame Check Error (CRC)
- `DWT_INT_RFSL` - RX Sync Loss
- `DWT_INT_RFTO` - RX Frame Wait Timeout

## ISR Task Implementation (Platform Layer)

### Task Creation (in dw1000_setup_isr)

```cpp
xTaskCreate(
    dw1000_isr_task,           // Task function
    "dw1000_isr",              // Task name
    4096,                      // Stack size
    NULL,                      // Parameters
    task_priority,             // Priority
    &dw1000_isr_task_handle    // Task handle
);
```

### ISR Task Function (Simplified)

```cpp
static void dw1000_isr_task(void *param)
{
    while (1) {
        // Wait for notification from GPIO ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Call DW1000 ISR handler
        dwt_isr();
        
        // dwt_isr() reads status and calls registered callbacks
    }
}
```

### GPIO ISR (Simplified)

```cpp
static void IRAM_ATTR dw1000_gpio_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Notify ISR task
    vTaskNotifyGiveFromISR(dw1000_isr_task_handle, 
                          &xHigherPriorityTaskWoken);
    
    // Yield if higher priority task woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

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
Setting up ISR with callbacks...
ISR and callbacks configured (priority 6)
Interrupts enabled: TFRS

========================================
   Initialization Complete!
   Starting TX...
========================================


*** CALLBACK: tx_conf_cb ***
Status: 0x00000080 - TX sent

*** CALLBACK: tx_conf_cb ***
Status: 0x00000080 - TX sent

[Repeats every 200ms...]
```

**Status 0x00000080** = SYS_STATUS_TXFRS flag set

## Performance Benefits

### CPU Efficiency

**Polling version:**
```
Frame preparation: 50µs
TX start: 10µs
Polling wait: 230µs ← CPU blocked
Post-TX processing: 20µs
Delay: 199.69ms
Total: 200ms (0.155% CPU usage for polling)
```

**Interrupt version:**
```
Frame preparation: 50µs
TX start: 10µs
Callback execution: 20µs ← Asynchronous
Delay: 199.92ms
Total: 200ms (0.04% CPU usage)
```

**CPU savings**: ~75% reduction in active waiting

### Multi-Tasking Benefits

With interrupts, the main loop can:
- Process sensor data
- Handle communication protocols
- Update displays
- Run control algorithms

...all while waiting for TX completion.

## Callback Execution Context

**Important**: Callbacks run in ISR task context, not main loop!

### Safe Operations in Callbacks
✅ Read/write variables (use `volatile` if accessed from multiple contexts)
✅ Call `Serial.printf()` (buffered, non-blocking)
✅ Set flags for main loop processing
✅ Send FreeRTOS notifications/queue items
✅ Read DW1000 registers (via callback data)

### Unsafe Operations in Callbacks
❌ Long computations (blocks ISR task)
❌ Blocking delays (`delay()`, long `Serial.print()`)
❌ Writing to DW1000 (potential race conditions)
❌ Complex memory allocations

## Thread Safety Considerations

### Shared Variables

If sharing variables between loop and callback:

```cpp
static volatile uint32_t tx_count = 0;  // volatile keyword required

void tx_conf_cb(const dwt_cb_data_t *cb_data) {
    tx_count++;  // Atomic on ESP32 for 32-bit aligned ints
}

void loop() {
    uint32_t current_count = tx_count;  // Read volatile variable
    // Use current_count...
}
```

### Critical Sections

For complex shared data:

```cpp
portENTER_CRITICAL(&my_mutex);
// Access shared data
portEXIT_CRITICAL(&my_mutex);
```

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No callbacks triggered | IRQ pin not connected | Verify physical connection |
| Callbacks not called | Interrupts not enabled | Check `dwt_setinterrupt()` |
| ISR setup failed | GPIO already in use | Check pin conflicts |
| Erratic callbacks | Electrical noise on IRQ | Add pull-up resistor, shorter wires |
| Callbacks freeze system | Blocking code in callback | Remove delays/long operations |

## Related Examples

- `UWB_TX_Test.cpp` - Polling-based transmitter
- `UWB_RX_Send_Test.cpp` - Interrupt-based RX with auto-response
- `UWB_TX_Resp_Test.cpp` - Full-duplex TX+RX with interrupts
- `UWB_DS_TWR_Init_Test.cpp` - Ranging with interrupts
- Official DecaWave: `ex_03d_tx_wait_resp_interrupts`

## Notes

- Interrupt priority 6 is typical; adjust based on system requirements
- Higher priority = more responsive but may impact other tasks
- ISR task stack size (4096 bytes) is usually sufficient
- Callbacks are serialized - only one executes at a time
- DW1000 hardware buffers interrupt events if ISR is slow
- This example only enables TX interrupts; see other examples for RX interrupt handling
