# UWB_two_way_comm_test - Two-Way Communication Examples

> **⚠️ WARNING: This documentation is AI-Generated Content (AIGC)**  
> Please verify all technical specifications, code implementations, and system designs independently. Do not rely solely on this documentation for critical system implementations.

## Overview

The `UWB_two_way_comm_test` folder contains interrupt-driven examples demonstrating bidirectional UWB communication. These examples show how to implement full-duplex communication where devices both transmit and receive, with automatic response handling.

## Files

1. **UWB_TX_Resp_Test.cpp** - Transmitter that waits for responses
2. **UWB_RX_Send_Test.cpp** - Receiver that automatically responds to valid frames

## Communication Pattern

```
TX_Resp Device                    RX_Send Device
      |                                  |
      |-------- Blink Frame ------------>|
      |   (seq: 0x00, payload)          |
      |                                  | [Validate]
      |                                  | [Copy seq number]
      |<------- Response Frame ----------|
      |   (seq: 0x00, response payload) |
      |                                  |
      [1 second delay]                   [Wait for next]
      |                                  |
      |-------- Blink Frame ------------>|
      |   (seq: 0x01, payload)          |
      |                                  |
      |<------- Response Frame ----------|
      |   (seq: 0x01, response payload) |
      |                                  |
```

## Hardware Requirements

- **2× ESP32-S3** microcontrollers
- **2× DW1000** UWB transceiver modules
- Complete SPI and GPIO connections
- **IRQ pin connection required** for interrupt operation

## Common Configuration

Both devices use identical RF configuration:

```cpp
static dwt_config_t dw1000_config = {
    1,                 /* Channel 1 (3.4944 GHz) */
    DWT_PRF_64M,       /* 64 MHz PRF */
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

## UWB_TX_Resp_Test.cpp - Transmitter with Response Waiting

### Purpose

Demonstrates full-duplex communication by:
1. Transmitting a blink frame
2. Automatically enabling receiver after transmission
3. Waiting for response with timeout
4. Processing received response
5. Repeating every 1 second

### State Machine

```cpp
typedef enum {
    STATE_IDLE,          // Ready to transmit
    STATE_TX_SENT,       // Frame transmitted, waiting for response
    STATE_RESP_RECEIVED  // Response received successfully
} comm_state_t;
```

### Frame Structure

**Transmitted Frame (Blink):**
```cpp
static uint8 tx_msg[] = {
    0xC5,       // Frame control: Blink frame
    0x00,       // Sequence number (increments)
    0xDE, 0xCA, 0x01, 0x23, 0x45, 0x67, 0x89,  // Payload
    0x00, 0x00  // CRC (auto)
};
```

**Expected Response Frame:**
```cpp
// Pattern: {0xC5, seq, 0xDE, 0xCA, 0x98, 0x76, 0x54, 0x32, 0x10, 0x00, 0x00}
```

### Timing Configuration

```cpp
#define TX_TO_RX_DELAY_UUS 500   // 500 µs delay before enabling RX
#define RX_RESP_TO_UUS 5000      // 5000 µs (5 ms) response timeout
```

### Key Features

#### Automatic RX After TX

```cpp
dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);  // Configure delay
dwt_setrxtimeout(RX_RESP_TO_UUS);           // Set timeout

// Start TX with response expected
dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
```

**What happens:**
1. Frame transmits
2. Hardware automatically waits 500 µs
3. Receiver enables automatically
4. Waits up to 5 ms for response
5. Callback triggered on RX success/timeout/error

#### Interrupt Callbacks

```cpp
void tx_conf_cb(const dwt_cb_data_t *cb_data);  // TX complete
void rx_ok_cb(const dwt_cb_data_t *cb_data);    // Response received
void rx_to_cb(const dwt_cb_data_t *cb_data);    // Response timeout
void rx_err_cb(const dwt_cb_data_t *cb_data);   // RX error
```

### Operation Flow

1. **IDLE State**: Write frame to TX buffer, start transmission with `DWT_RESPONSE_EXPECTED`
2. **TX Callback**: Transmission complete → STATE_TX_SENT
3. **Wait for Response**: Hardware automatically enables RX after delay
4. **RX Success Callback**: 
   - Validate response frame pattern
   - Check sequence number matches
   - Transition to STATE_RESP_RECEIVED
5. **RX Timeout Callback**: No response within 5ms → reset to IDLE
6. **RX Error Callback**: Reception error → reset to IDLE
7. **Loop Delay**: 1 second before next transmission

### Expected Serial Output

```
========================================
   TX/Wait Response Test (Interrupts)
========================================

[Initialization messages...]
RX after TX delay: 500 uus, RX timeout: 5000 uus

========================================
   Initialization Complete!
   Starting TX/RX loop...
========================================

=== Stats: TX: 0, RX OK: 0, RX TO: 0, RX Err: 0 | TX Count: 0 ===
[LOOP] Sent frame #1, seq 0
[TX_CONF] Frame sent, waiting for response
[RX_OK] Response received, seq 0

=== Stats: TX: 1, RX OK: 1, RX TO: 0, RX Err: 0 | TX Count: 1 ===
[LOOP] Sent frame #2, seq 1
[TX_CONF] Frame sent, waiting for response
[RX_OK] Response received, seq 1

=== Stats: TX: 2, RX OK: 2, RX TO: 0, RX Err: 0 | TX Count: 2 ===
...
```

### With Timeout (No Responder)

```
[LOOP] Sent frame #1, seq 0
[TX_CONF] Frame sent, waiting for response
[RX_TO] Response timeout

=== Stats: TX: 1, RX OK: 0, RX TO: 1, RX Err: 0 | TX Count: 1 ===
```

## UWB_RX_Send_Test.cpp - Receiver with Auto-Response

### Purpose

Demonstrates automatic response transmission by:
1. Continuously listening for frames
2. Validating received frames
3. Automatically sending response with matching sequence number
4. Returning to listen mode

### Frame Validation

**Expected Received Frame Pattern:**
```cpp
// Bytes: C5 XX DE CA 01 23 45 67 89 ...
// Validates bytes [2-8] match specific pattern
```

**Response Frame:**
```cpp
static uint8 tx_msg[] = {
    0xC5,       // Frame control
    0x00,       // Sequence number (copied from RX)
    0xDE, 0xCA, 0x98, 0x76, 0x54, 0x32, 0x10,  // Response payload
    0x00, 0x00  // CRC
};
```

### Timing Configuration

```cpp
#define RX_TO_TX_DELAY_US 1000  // 1 ms delay before sending response
```

### Key Features

#### Frame Validation Logic

```cpp
bool is_valid = (cb_data->datalength >= 9) &&
                (rx_buffer[2] == 0xDE) &&
                (rx_buffer[3] == 0xCA) &&
                (rx_buffer[4] == 0x01) &&
                (rx_buffer[5] == 0x23) &&
                (rx_buffer[6] == 0x45) &&
                (rx_buffer[7] == 0x67) &&
                (rx_buffer[8] == 0x89);
```

Only responds to frames with correct payload pattern.

#### Response Transmission

```cpp
if (is_valid) {
    // Copy sequence number from received frame
    tx_msg[RESP_FRAME_SN_IDX] = rx_buffer[BLINK_FRAME_SN_IDX];
    
    // Write and send response
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
    dwt_writetxfctrl(sizeof(tx_msg), 0, 0);
    
    delayMicroseconds(RX_TO_TX_DELAY_US);  // Brief delay
    
    dwt_starttx(DWT_START_TX_IMMEDIATE);
}
```

**Note**: Uses simple `delayMicroseconds()` instead of delayed TX scheduling. For production, consider using `dwt_setdelayedtrxtime()` with microcode enabled.

#### Automatic RX Re-enable

After sending response, TX callback automatically re-enables receiver:

```cpp
void tx_conf_cb(const dwt_cb_data_t *cb_data) {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);  // Back to listening
}
```

### Operation Flow

1. **Startup**: Enable receiver immediately
2. **RX Callback**: Frame received → validate pattern
3. **Valid Frame**: 
   - Copy sequence number
   - Prepare response frame
   - Delay 1 ms
   - Transmit response
   - Increment response counter
4. **Invalid Frame**: 
   - Log validation failure
   - Re-enable receiver
5. **TX Callback**: Response sent → re-enable receiver
6. **RX Error Callback**: Error occurred → re-enable receiver
7. **Continuous Loop**: Always listening

### Expected Serial Output

```
========================================
   RX/Send Response Test (Interrupts)
========================================

[Initialization messages...]

========================================
   Initialization Complete!
   Starting RX/Response loop...
========================================

=== Stats: RX OK: 0, RX Err: 0, TX: 0 | Responses: 0 ===
[RX_OK] #1 Valid frame received, seq 0, sending response
[TX_CONF] Response sent

=== Stats: RX OK: 1, RX Err: 0, TX: 1 | Responses: 1 ===
[RX_OK] #2 Valid frame received, seq 1, sending response
[TX_CONF] Response sent

=== Stats: RX OK: 2, RX Err: 0, TX: 2 | Responses: 2 ===
...
```

### With Invalid Frames

```
[RX_OK] #5 Invalid frame received (pattern mismatch)

=== Stats: RX OK: 5, RX Err: 0, TX: 4 | Responses: 4 ===
```

## Key Functions

### DW1000 Driver Functions

#### TX/RX Control
- `dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)` - TX with auto-RX
- `dwt_rxenable(DWT_START_RX_IMMEDIATE)` - Enable receiver
- `dwt_setrxaftertxdelay(delay_uus)` - Set RX enable delay after TX
- `dwt_setrxtimeout(timeout_uus)` - Set RX timeout

#### Data Access
- `dwt_writetxdata()` - Write frame to TX buffer
- `dwt_readrxdata()` - Read received frame from RX buffer

### Platform Functions
- `dw1000_setup_isr()` - Setup interrupt callbacks
- `dwt_setinterrupt()` - Enable specific interrupt sources

### Interrupt Flags

**TX_Resp_Test enables:**
- `DWT_INT_TFRS` - TX Frame Sent
- `DWT_INT_RFCG` - RX Frame Good
- `DWT_INT_RFTO` - RX Timeout
- `DWT_INT_RPHE` - RX PHY Header Error
- `DWT_INT_RFCE` - RX Frame Check Error
- `DWT_INT_RFSL` - RX Sync Loss
- `DWT_INT_RXOVRR` - RX Overrun
- `DWT_INT_RXPTO` - RX Preamble Timeout
- `DWT_INT_SFDT` - SFD Timeout

**RX_Send_Test enables:**
- `DWT_INT_TFRS` - TX Frame Sent
- `DWT_INT_RFCG` - RX Frame Good
- `DWT_INT_RPHE` - RX PHY Header Error
- `DWT_INT_RFCE` - RX Frame Check Error
- `DWT_INT_RFSL` - RX Sync Loss
- `DWT_INT_SFDT` - SFD Timeout

## Timing Diagrams

### Successful Communication Cycle

```
Time (µs)    TX_Resp Device              RX_Send Device
    0        Start TX
  ~230       TX complete
  230        [500 µs delay]              
  730        Enable RX                   
            
            Meanwhile on RX_Send:
    0                                     Listening
  ~230                                    RX complete
  230                                     [Validate]
 1230                                     [1ms delay]
 1230                                     Start TX
 1460                                     TX complete
 1460                                     Enable RX
            
Back to TX_Resp:
 1460        Receive response
 1460        [Process response]
            
           [1 second delay]
            
1001460      Start next TX
```

### With Timeout (No Response)

```
Time (µs)    TX_Resp Device
    0        Start TX
  ~230       TX complete
  730        Enable RX
 5730        Timeout (5000 µs elapsed)
 5730        [Reset to IDLE]
            
1000000      Start next TX
```

## Performance Characteristics

- **Round-trip time**: ~1.5 ms (TX → Response RX)
- **TX air time**: ~230 µs per 11-byte frame
- **Processing delay**: 1 ms (RX_Send device)
- **Communication rate**: 1 Hz (limited by TX_Resp 1-second delay)
- **Success rate**: ~99.9% at short range (<10m, clear LOS)

## Statistics Monitoring

### TX_Resp Device Tracks
- `tx_conf_count` - Transmissions completed
- `rx_ok_count` - Responses received
- `rx_to_count` - Response timeouts
- `rx_err_count` - RX errors
- `tx_count` - Total transmission attempts

### RX_Send Device Tracks
- `rx_ok_count` - Frames received (valid + invalid)
- `rx_err_count` - RX errors
- `tx_conf_count` - Responses sent
- `resp_count` - Valid frames responded to

## Troubleshooting

| Issue | TX_Resp Output | RX_Send Output | Solution |
|-------|----------------|----------------|----------|
| No communication | All RX TO | No RX OK | Check configurations match, both powered |
| Partial communication | Some RX OK, some TO | Responses sent | Check range, interference |
| No responses | All RX TO | RX OK but no TX | Check frame validation logic |
| Wrong responses | RX OK but invalid | TX sent | Verify payload patterns match |
| Frequent errors | High RX Err count | High RX Err count | Check signal quality, reduce range |

## Comparison with Other Examples

| Feature | Basic TX/RX | Two-Way Comm | DS-TWR |
|---------|-------------|--------------|--------|
| Communication | One-way | Bidirectional | Bidirectional |
| Responses | None | Simple ack | Timed responses |
| Ranging | No | No | Yes |
| Timestamps | No | No | Yes |
| Microcode | Optional | Not required | Required |
| Complexity | Low | Medium | High |
| Use Case | Telemetry | Command/response | Distance measurement |

## Use Cases

### Suitable For
- Command/acknowledgment protocols
- Request/reply communication
- Link quality testing
- Network discovery/beacon responses
- Simple bidirectional data exchange

### Not Suitable For
- Ranging/positioning (use DS-TWR instead)
- High-throughput data transfer
- Multi-device networks (no addressing/arbitration)
- Continuous data streaming

## Modifications

### Increase Communication Rate

In `UWB_TX_Resp_Test.cpp`:
```cpp
vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz instead of 1 Hz
```

**Note**: Ensure response timeout allows sufficient time.

### Add Addressing

Add device address to frame:
```cpp
static uint8 tx_msg[] = {
    0xC5, 0x00,       // Frame control + sequence
    0x01,             // Source address
    0x02,             // Destination address
    0xDE, 0xCA, ...   // Payload
};
```

Validate address in RX callback before responding.

### Use Delayed TX (Recommended)

In `UWB_RX_Send_Test.cpp`, replace `delayMicroseconds()`:

```cpp
// Initialize with microcode
dwt_initialise(DWT_LOADUCODE);

// In rx_ok_cb:
uint64 resp_rx_ts = get_rx_timestamp_u64();
uint32 resp_tx_time = (resp_rx_ts + (RX_TO_TX_DELAY_US * UUS_TO_DWT_TIME)) >> 8;
dwt_setdelayedtrxtime(resp_tx_time);
dwt_starttx(DWT_START_TX_DELAYED);
```

Benefits: More precise timing, doesn't block ISR task.

## Debug Features

### Debug Macros

```cpp
#define DEBUG_STATUS_STATE 0  // Register dumps
#define DEBUG_CALLBACKS 1     // Callback messages
```

Enable for detailed debugging information.

### Status/State Printing

Function `print_status_state()` displays:
- SYS_STATUS register (all status flags)
- SYS_STATE register (device state machine)

Useful for diagnosing communication issues.

## Related Examples

- `UWB_TX_Test.cpp` / `UWB_RX_Test.cpp` - One-way communication
- `UWB_TX_ISR_Test.cpp` - Interrupt-based TX only
- `UWB_DS_TWR_*` - Ranging with precise timing
- Official DecaWave: `ex_03b_rx_send_resp`, `ex_03d_tx_wait_resp_interrupts`

## Notes

- **Microcode not required** for basic communication (only needed for ranging)
- Both examples use automatic SPI bus acquisition (default behavior)
- Sequence numbers must match between request and response
- Frame validation prevents responding to unintended frames
- `delayMicroseconds()` in callback is acceptable for short delays (<1ms)
- For production, consider using delayed TX scheduling instead of blocking delays
- Examples demonstrate ping-pong communication pattern
- Can be extended to multi-hop or broadcast with modifications
