# UWB_DS_DWR_Test - Double-Sided Two-Way Ranging Examples

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

The `UWB_DS_DWR_Test` folder contains interrupt-driven examples for Double-Sided Two-Way Ranging (DS-TWR), an accurate distance measurement protocol using UWB. This implementation requires two devices: an **initiator** and a **responder** working together to calculate precise distances.

## Files

1. **UWB_DS_TWR_Init_Test.cpp** - Initiator device (starts ranging exchanges)
2. **UWB_DS_TWR_Resp_Test.cpp** - Responder device (responds and calculates distance)

## What is Double-Sided Two-Way Ranging?

DS-TWR is a time-of-flight (ToF) distance measurement protocol that eliminates clock drift errors by using timestamps from both devices.

### Message Exchange Sequence

```
Initiator                    Responder
    |                             |
    |------ POLL Message -------->|  T_poll_tx
    |                             |  T_poll_rx
    |                             |
    |<---- RESPONSE Message ------|  T_resp_tx
    |  T_resp_rx                  |
    |                             |
    |------ FINAL Message ------->|  T_final_tx
    |                             |  T_final_rx
    |                             |
    |                        [Calculate Distance]
```

### Timestamps Used

**Initiator Records:**
- `T_poll_tx` - Poll transmission time
- `T_resp_rx` - Response reception time  
- `T_final_tx` - Final transmission time

**Responder Records:**
- `T_poll_rx` - Poll reception time
- `T_resp_tx` - Response transmission time
- `T_final_rx` - Final reception time

### Distance Calculation

The responder calculates distance using:

```
Round_A = T_resp_rx - T_poll_tx
Reply_A = T_final_tx - T_resp_rx

Round_B = T_final_rx - T_resp_tx  
Reply_B = T_resp_tx - T_poll_rx

ToF = (Round_A × Round_B - Reply_A × Reply_B) / (Round_A + Round_B + Reply_A + Reply_B)

Distance = ToF × Speed_of_Light
```

This double-sided approach cancels out clock frequency offsets between devices.

## Hardware Requirements

- **2× ESP32-S3** microcontrollers
- **2× DW1000** UWB transceiver modules
- Complete SPI and GPIO connections on both devices
- **IRQ pin connection required** for interrupt operation

## Common Configuration

Both initiator and responder use identical DW1000 configuration:

```cpp
static dwt_config_t dw1000_config = {
    1,                 /* Channel 1 (3.4944 GHz) */
    DWT_PRF_64M,       /* 64 MHz PRF for ranging */
    DWT_PLEN_128,      /* 128-symbol preamble */
    DWT_PAC8,          /* PAC size 8 */
    9,                 /* TX preamble code 9 */
    9,                 /* RX preamble code 9 */
    1,                 /* Non-standard SFD */
    DWT_BR_850K,       /* 850 kbps data rate */
    DWT_PHRMODE_STD,   /* Standard PHY header */
    (128 + 8 + 64 - 8) /* SFD timeout */
};
```

### Antenna Delay Calibration

```cpp
#define TX_ANT_DLY 16436  // Default for 64 MHz PRF
#define RX_ANT_DLY 16436
```

**⚠️ Important**: These are default values. For **accurate ranging**, you must calibrate antenna delays for your specific hardware. Uncalibrated delays can cause errors of several meters.

## UWB_DS_TWR_Init_Test.cpp - Initiator

### Purpose

Initiates ranging exchanges by:
1. Sending POLL message
2. Waiting for RESPONSE
3. Sending FINAL message with timestamps
4. Repeating every 1 second

### State Machine

```cpp
typedef enum {
    STATE_IDLE,          // Ready to start new ranging
    STATE_POLL_SENT,     // Poll transmitted, waiting for response
    STATE_RESP_RECEIVED  // Response received, sending final
} ds_twr_init_state_t;
```

### Frame Messages

```cpp
// Poll: Initiator → Responder
static uint8 tx_poll_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0
};

// Response: Responder → Initiator (expected)
static uint8 rx_resp_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0
};

// Final: Initiator → Responder (with timestamps)
static uint8 tx_final_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23,
    0, 0, 0, 0,  // Poll TX timestamp
    0, 0, 0, 0,  // Response RX timestamp
    0, 0, 0, 0,  // Final TX timestamp
    0, 0         // CRC
};
```

### Timing Parameters

```cpp
#define POLL_TX_TO_RESP_RX_DLY_UUS 900   // Delay after poll TX before RX enable
#define RESP_RX_TO_FINAL_TX_DLY_UUS 1000 // Delay from resp RX to final TX
#define RESP_RX_TIMEOUT_UUS 2000         // Response timeout (2ms)
#define RNG_DELAY_MS 1000                // Delay between ranging exchanges
```

### Operation Flow

1. **IDLE State**: Send POLL message with `DWT_RESPONSE_EXPECTED` flag
2. **TX Callback**: Poll sent → transition to POLL_SENT state
3. **POLL_SENT State**: 
   - Calculate delayed RX enable time
   - Set RX timeout
   - Enable receiver with delayed start
4. **RX Callback**: Validate RESPONSE message → transition to RESP_RECEIVED
5. **RESP_RECEIVED State**:
   - Retrieve response RX timestamp
   - Calculate delayed final TX time  
   - Embed timestamps in FINAL message
   - Schedule delayed transmission
6. **TX Callback**: Final sent → return to IDLE
7. **Timeout/Error Callbacks**: Reset to IDLE state

### Interrupt Callbacks

```cpp
void tx_conf_cb(const dwt_cb_data_t *cb_data);  // TX confirmation
void rx_ok_cb(const dwt_cb_data_t *cb_data);    // RX success
void rx_to_cb(const dwt_cb_data_t *cb_data);    // RX timeout
void rx_err_cb(const dwt_cb_data_t *cb_data);   // RX error
```

### Expected Serial Output

```
========================================
   DS-TWR Initiator Test (Interrupts)
========================================

[Initialization messages...]

=== Stats: TX: 0, RX OK: 0, RX TO: 0, RX Err: 0 | Ranging: 0 ===
[LOOP] Initiated ranging #1, sent POLL
[TX_CONF] POLL sent, waiting for RESPONSE
[RX_OK] RESPONSE received, sending FINAL
[TX_CONF] FINAL sent, ranging complete

=== Stats: TX: 2, RX OK: 1, RX TO: 0, RX Err: 0 | Ranging: 1 ===
[LOOP] Initiated ranging #2, sent POLL
...
```

## UWB_DS_TWR_Resp_Test.cpp - Responder

### Purpose

Responds to ranging exchanges and calculates distance:
1. Waits for POLL message
2. Sends RESPONSE with delay
3. Waits for FINAL message
4. Calculates and displays distance

### State Machine

```cpp
typedef enum {
    STATE_WAITING_POLL,   // Waiting for poll message
    STATE_POLL_RECEIVED,  // Poll received, sending response
    STATE_RESP_SENT       // Response sent, waiting for final
} ds_twr_resp_state_t;
```

### Frame Messages

```cpp
// Poll: Initiator → Responder (expected)
static uint8 rx_poll_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0
};

// Response: Responder → Initiator
static uint8 tx_resp_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0
};

// Final: Initiator → Responder (expected, with timestamps)
static uint8 rx_final_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23,
    0, 0, 0, 0,  // Poll TX timestamp (from initiator)
    0, 0, 0, 0,  // Response RX timestamp (from initiator)
    0, 0, 0, 0,  // Final TX timestamp (from initiator)
    0, 0         // CRC
};
```

### Timing Parameters

```cpp
#define POLL_RX_TO_RESP_TX_DLY_UUS 1000  // Delay from poll RX to response TX
#define RESP_TX_TO_FINAL_RX_DLY_UUS 900  // Delay after resp TX before RX enable
#define FINAL_RX_TIMEOUT_UUS 2000        // Final message timeout
```

### Operation Flow

1. **WAITING_POLL State**: Receiver continuously enabled
2. **RX Callback**: Validate POLL message → transition to POLL_RECEIVED
3. **POLL_RECEIVED State**:
   - Retrieve poll RX timestamp
   - Calculate delayed response TX time
   - Schedule delayed RESPONSE transmission with `DWT_RESPONSE_EXPECTED`
4. **TX Callback**: Response sent → transition to RESP_SENT
5. **RESP_SENT State**:
   - Calculate delayed RX enable time
   - Set final RX timeout
   - Enable receiver with delayed start
6. **RX Callback**: Receive FINAL message with timestamps
7. **Distance Calculation**:
   - Extract initiator's timestamps from FINAL message
   - Combine with local timestamps (poll_rx, resp_tx, final_rx)
   - Compute time-of-flight using DS-TWR formula
   - Convert to distance in meters
   - Display result on serial
8. **Return to WAITING_POLL**: Start listening for next poll

### Distance Calculation Implementation

```cpp
// Extract timestamps from FINAL message
uint32 poll_tx_ts_32, resp_rx_ts_32, final_tx_ts_32;
final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts_32);
final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts_32);
final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_32);

// Compute time differences
int32 rtd_init = resp_rx_ts_32 - poll_tx_ts_32;
int32 rtd_resp = final_rx_ts_32 - resp_tx_ts_32;
int32 dtof = ((rtd_init - rtd_resp) / 2);

// Clock offset ratio
float clockOffsetRatio = ((float)dwt_readcarrierintegrator()) * 
                         (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_1);

// Convert to time-of-flight (seconds)
tof = dtof * DWT_TIME_UNITS;
distance = tof * SPEED_OF_LIGHT;
```

### Expected Serial Output

```
========================================
   DS-TWR Responder Test (Interrupts)
========================================

[Initialization messages...]

=== Stats: TX: 0, RX OK: 0, RX TO: 0, RX Err: 0 | Ranging: 0, Distance: 0.000 m ===
[RX_OK] POLL received, sending RESPONSE
[TX_CONF] RESPONSE sent, waiting for FINAL
[RX_OK] FINAL received - RANGE: 2.347 m

=== Stats: TX: 1, RX OK: 2, RX TO: 0, RX Err: 0 | Ranging: 1, Distance: 2.347 m ===
[RX_OK] POLL received, sending RESPONSE
[TX_CONF] RESPONSE sent, waiting for FINAL
[RX_OK] FINAL received - RANGE: 2.351 m

=== Stats: TX: 2, RX OK: 4, RX TO: 0, RX Err: 0 | Ranging: 2, Distance: 2.351 m ===
...
```

## Key Functions

### Platform Functions (ESP32 Port)

- `dw1000_setup_isr()` - Setup interrupt handling with all 4 callbacks
- `dw1000_auto_bus_acquisition(false)` - Disable automatic SPI bus locking
- `dw1000_spi_acquire_bus()` - Manually acquire SPI bus before DW1000 operations
- `dw1000_spi_release_bus()` - Release SPI bus after operations

### DW1000 Driver Functions

#### Ranging-Specific
- `dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)` - TX with RX enable after
- `dwt_setdelayedtrxtime()` - Schedule future TX/RX
- `dwt_setrxtimeout()` - Set RX timeout for expected message
- `dwt_readtxtimestamp()` - Read TX timestamp
- `dwt_readrxtimestamp()` - Read RX timestamp
- `dwt_readcarrierintegrator()` - Read clock offset for compensation

#### General
- `dwt_forcetrxoff()` - Force transceiver to idle
- `dwt_rxenable()` - Enable receiver (immediate or delayed)

### Timestamp Helpers

```cpp
// Get 64-bit TX timestamp
static uint64 get_tx_timestamp_u64(void) {
    uint8 ts_tab[5];
    dwt_readtxtimestamp(ts_tab);
    return (uint64)ts_tab[0] + 
           ((uint64)ts_tab[1] << 8) + 
           ((uint64)ts_tab[2] << 16) + 
           ((uint64)ts_tab[3] << 24) + 
           ((uint64)ts_tab[4] << 32);
}

// Similar for RX: get_rx_timestamp_u64()
```

## SPI Bus Management

These examples disable automatic SPI bus acquisition:

```cpp
```

**Why?** Prevents deadlocks when ISR callbacks need SPI access. Callbacks manually acquire/release bus:

```cpp
void rx_ok_cb(const dwt_cb_data_t *cb_data) {
    decaIrqStatus_t stat = decamutexon();
    // ... DW1000 operations ...
    decamutexoff(stat);
}
```

## Accuracy and Calibration

### Default Accuracy

With default antenna delays: **±0.5 to 2 meters** typical error

### After Calibration

With calibrated antenna delays: **±10 cm** achievable

### Calibration Process

1. Place devices at known distance (e.g., 10.00 meters measured with tape)
2. Run DS-TWR and note measured distance
3. Calculate antenna delay correction
4. Iterate until measured distance matches actual
5. Update `TX_ANT_DLY` and `RX_ANT_DLY` values

### Factors Affecting Accuracy

- **Antenna delay calibration** - Most critical factor
- **Clock drift** - Minimized by DS-TWR algorithm
- **Multipath propagation** - Reflections from walls/objects
- **LOS conditions** - Best accuracy with clear line-of-sight
- **Temperature** - Affects crystal frequency slightly

## Troubleshooting

| Issue | Initiator Output | Responder Output | Solution |
|-------|------------------|------------------|----------|
| No communication | RX timeout errors | No POLL received | Check device configurations match, verify both powered on |
| Distance wildly incorrect | Ranging completes | Shows unrealistic distance | Calibrate antenna delays |
| Intermittent timeouts | Occasional RX TO | Occasional missing FINAL | Increase timeout values, check for interference |
| Always timeout | All RX TO | Some POLL received | Check timing parameters, ensure delays are sufficient |
| Distance drifts | Ranging works | Distance varies ±meters | Check for multipath, move to open area |

## Performance Characteristics

- **Ranging rate**: 1 Hz (one distance measurement per second)
- **Time per ranging exchange**: ~3-5 milliseconds
- **Message overhead**: 3 messages per ranging (POLL, RESPONSE, FINAL)
- **CPU usage**: Low (interrupt-driven, most time spent idle)
- **Power consumption**: Moderate (transmitter duty cycle ~0.3%)

## Debug Features

### Debug Macros

```cpp
#define DEBUG_STATUS_STATE 0  // Enable detailed register dumps
#define DEBUG_CALLBACKS 1     // Enable callback debug prints
```

Set to `1` to enable verbose debugging output showing status/state registers at each step.

### Statistics Monitoring

Both examples track:
- `tx_conf_count` - TX confirmations received
- `rx_ok_count` - Successful RX events
- `rx_to_count` - RX timeout events
- `rx_err_count` - RX error events  
- `ranging_count` - Successful ranging exchanges (responder)

## Related Examples

- `UWB_TX_Resp_Test.cpp` / `UWB_RX_Send_Test.cpp` - Simpler two-way communication
- Official DecaWave: `ex_05a_ds_twr_init` / `ex_05b_ds_twr_resp`
- Alternative: `ex_06a_ss_twr_init` / `ex_06b_ss_twr_resp` (single-sided ranging, less accurate)

## Notes

- Responder calculates distance; initiator does not
- Sequence numbers in messages are for debugging/logging
- `DWT_LOADUCODE` is required for ranging (loads LDE microcode)
- Both devices must use identical RF configuration
- Works at ranges from ~10 cm to ~100+ meters (depends on environment)
- For best results: clear line-of-sight, minimize reflective surfaces
