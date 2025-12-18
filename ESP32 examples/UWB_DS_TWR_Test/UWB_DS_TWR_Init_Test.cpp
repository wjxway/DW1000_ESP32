/**
 * @file UWB_DS_TWR_Init_Test.cpp
 * @brief Double-sided Two-Way Ranging (DS-TWR) Initiator using interrupts for ESP32-S3
 *
 * This application acts as the initiator in a DS-TWR distance measurement exchange.
 * It sends a "poll" frame, waits for a "response" from the responder, then sends a
 * "final" message containing all timestamps. The responder calculates the distance.
 * All RX/TX events are handled through ISR callbacks.
 * Based on ex_05a_ds_twr_init rewritten to use interrupt-driven operation.
 */

#include <Arduino.h>
#include <HardwareDefs.hpp>
#include <Blink.hpp>
#include <stdio.h>
#include <string.h>

// Include DW1000 driver
extern "C"
{
#include "platform/deca_spi.h"
#include "platform/deca_gpio.h"
#include "platform/deca_debug.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
}

/* Debug macros */
#define DEBUG_STATUS_STATE 0 // Enable/disable status and state register printing
#define DEBUG_CALLBACKS 1    // Enable/disable callback debug prints

/* Configuration for optimal operation */
static dwt_config_t dw1000_config = {
    1,                 /* Channel number. */
    DWT_PRF_64M,       /* Pulse repetition frequency. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    1,                 /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    (128 + 8 + 64 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process.
 * Poll message:  initiator -> responder (trigger ranging)
 * Response message: responder -> initiator (continue ranging)
 * Final message: initiator -> responder (complete ranging with timestamps) */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. */
/* This is the delay from the end of the frame transmission to the enable of the receiver. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 500
/* This is the delay from Frame RX timestamp to TX reply timestamp. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 1000
/* Receive response timeout. */
#define RESP_RX_TIMEOUT_UUS 2000

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* State machine for DS-TWR initiator */
typedef enum
{
    STATE_IDLE,
    STATE_POLL_SENT,
    STATE_RESP_RECEIVED,
} ds_twr_init_state_t;

static volatile ds_twr_init_state_t current_state = STATE_IDLE;

/* Callback statistics */
static volatile uint32_t tx_conf_count = 0;
static volatile uint32_t rx_ok_count = 0;
static volatile uint32_t rx_to_count = 0;
static volatile uint32_t rx_err_count = 0;

/* Ranging count */
static volatile uint32_t ranging_count = 0;

/* Forward declarations of callback functions */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

/* Forward declarations of helper functions */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);

static void print_status_state(const char *prefix)
{
#if DEBUG_STATUS_STATE
    decaIrqStatus_t stat = decamutexon();
    sys_status_reg_t status;
    sys_state_reg_t state;
    char status_buf[512], state_buf[512];
    deca_get_sys_status(&status);
    deca_get_sys_state(&state);
    decamutexoff(stat);
    deca_get_status_string(&status, status_buf, sizeof(status_buf));
    deca_get_state_string(&state, state_buf, sizeof(state_buf));
    Serial.printf("[%s] Status: %s\n", prefix, status_buf);
    Serial.printf("[%s] State: %s\n", prefix, state_buf);
#endif
}

/* reset state and go back to idle state */
static void reset_state_and_idle()
{
    decaIrqStatus_t stat = decamutexon();
    current_state = STATE_IDLE;
    dwt_setrxtimeout(0);
    dwt_forcetrxoff();
    decamutexoff(stat);
}

/**
 * @brief Arduino setup function
 */
void setup()
{
    /* Initialize serial communication */
    Serial.begin(115200);
    vTaskDelay(1000);

    /* Blink LED to indicate start */
    Blink(500, 3, true, true, true);

    Serial.println("\n========================================");
    Serial.println("   DS-TWR Initiator Test (Interrupts)");
    Serial.println("========================================\n");

    /* Setup IMU CS pin to high (deselect IMU) */
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);
    Serial.println("IMU CS set to HIGH");

    /* Configure SPI bus */
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
        .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
        .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0};

    /* Initialize DW1000 SPI */
    if (dw1000_spi_init(SPI2_HOST, (gpio_num_t)UWB_CS_PIN, &spi_bus_cfg) != 0)
    {
        Serial.println("ERROR: SPI init failed!");
        while (1)
            ;
    }

    /* Configure DW1000 GPIO */
    if (dw1000_gpio_init((gpio_num_t)UWB_RST_PIN, (gpio_num_t)UWB_IRQ_PIN, GPIO_NUM_NC) != 0)
    {
        Serial.println("ERROR: GPIO init failed!");
        while (1)
            ;
    }

    /* Reset DW1000 */
    dw1000_hard_reset();
    dw1000_spi_fix_bug();
    vTaskDelay(5);

    /* Initialize DW1000 with LDE microcode for ranging */
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        Serial.println("ERROR: dwt_initialise failed!");
        while (1)
            ;
    }

    /* Set SPI to high speed */
    spi_set_rate_high();

    /* Configure DW1000 */
    dwt_configure(&dw1000_config);
    Serial.println("SPI, GPIO, and DW1000 initialized and configured successfully");

    /* Apply antenna delay values */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    Serial.printf("Antenna delays set - RX: %u, TX: %u\n", RX_ANT_DLY, TX_ANT_DLY);

    // dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    Serial.printf("RX after TX delay: %u uus, RX timeout: %u uus\n",
                  POLL_TX_TO_RESP_RX_DLY_UUS, RESP_RX_TIMEOUT_UUS);

    /* Setup interrupt handling with callbacks */
    Serial.println("Setting up ISR with callbacks...");
    if (dw1000_setup_isr(6, &tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb) != 0)
    {
        Serial.println("ERROR: dw1000_setup_isr failed!");
        while (1)
            ;
    }
    Serial.println("ISR and callbacks configured (priority 6)");

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors) */
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXOVRR | DWT_INT_RXPTO | DWT_INT_SFDT, 1);
    Serial.println("Interrupts enabled: TFRS, RFCG, RPHE, RFCE, RFSL, RFTO, RXOVRR, RXPTO, SFDT");

    Serial.println("\n========================================");
    Serial.println("   Initialization Complete!");
    Serial.println("   Starting DS-TWR Initiator loop...");
    Serial.println("========================================\n");

    // disable auto-bus acquisition to allow manual control of the SPI bus locks
}

/**
 * @brief Arduino loop function - initiates ranging exchanges periodically
 */
void loop()
{
    /* Print callback statistics */
    Serial.printf("\n=== Stats: TX: %lu, RX OK: %lu, RX TO: %lu, RX Err: %lu | Ranging: %lu ===\n",
                  tx_conf_count, rx_ok_count, rx_to_count, rx_err_count, ranging_count);
    print_status_state("LOOP");

    /* Reset state and start a new ranging exchange */
    current_state = STATE_IDLE;

    /* Write poll frame data to DW1000 and prepare transmission */
    decaIrqStatus_t stat = decamutexon();

    // refresh - force TRX off before starting new transmission
    dwt_forcetrxoff();

    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* ranging bit set */

    /* Set expected delay and timeout for response message reception */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Start transmission with response expected */
    int tx_ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    decamutexoff(stat);

    if (tx_ret == DWT_SUCCESS)
    {
        frame_seq_nb++;
#if DEBUG_CALLBACKS
        Serial.printf("[LOOP] Initiated ranging #%lu, sent POLL\n", ranging_count + 1);
#endif
    }
    else
    {
#if DEBUG_CALLBACKS
        Serial.printf("[LOOP] ERROR: dwt_starttx failed: %d\n", tx_ret);
#endif
    }

    /* Wait before next ranging exchange */
    vTaskDelay(pdMS_TO_TICKS(RNG_DELAY_MS));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    rx_ok_count++;

    decaIrqStatus_t stat = decamutexon();

    if (current_state != STATE_POLL_SENT)
    {
        reset_state_and_idle();
#if DEBUG_CALLBACKS
        Serial.printf("[RX_OK] #%lu Unexpected state %d, ignoring\n", rx_ok_count, current_state);
#endif
        print_status_state("RX_OK");

        decamutexoff(stat);
        return;
    }

    /* A frame has been received, copy it to our local buffer */
    if (cb_data->datalength <= RX_BUF_LEN)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);

        /* Check that the frame is the expected response from the responder.
         * Clear sequence number for validation. */
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            current_state = STATE_RESP_RECEIVED;

            /* Retrieve poll transmission and response reception timestamp */
            poll_tx_ts = get_tx_timestamp_u64();
            resp_rx_ts = get_rx_timestamp_u64();

            /* Compute final message transmission time */
            uint32 final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(final_tx_time);

            /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay */
            final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            /* Write all timestamps in the final message */
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

            /* Write and send final message */
            tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
            dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* ranging bit set */

            int ret = dwt_starttx(DWT_START_TX_DELAYED);

            if (ret == DWT_SUCCESS)
            {
                frame_seq_nb++;
#if DEBUG_CALLBACKS
                Serial.printf("[RX_OK] #%lu RESP received, RX ts: %llu, sending FINAL\n", rx_ok_count, resp_rx_ts);

#endif
                print_status_state("RX_OK");
                decamutexoff(stat);
                return;
            }
            else
            {
                reset_state_and_idle();
#if DEBUG_CALLBACKS
                Serial.printf("[RX_OK] #%lu ERROR: Final TX failed (late): %d\n", rx_ok_count, ret);
#endif
                print_status_state("RX_OK");
                decamutexoff(stat);
                return;
            }
        }
        else
        {
            reset_state_and_idle();
#if DEBUG_CALLBACKS
            Serial.printf("[RX_OK] #%lu Response frame validation FAILED\n", rx_ok_count);
#endif
            print_status_state("RX_OK");
            decamutexoff(stat);
            return;
        }
    }
    else
    {
        reset_state_and_idle();
#if DEBUG_CALLBACKS
        Serial.printf("[RX_OK] #%lu Frame too long (%u > %d)\n", rx_ok_count, cb_data->datalength, RX_BUF_LEN);
#endif
        print_status_state("RX_OK");
        decamutexoff(stat);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    rx_to_count++;

    decaIrqStatus_t stat = decamutexon();
    reset_state_and_idle();
#if DEBUG_CALLBACKS
    Serial.printf("[RX_TO] #%lu Timeout occurred\n", rx_to_count);
#endif
    print_status_state("RX_TO");
    decamutexoff(stat);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    rx_err_count++;

    decaIrqStatus_t stat = decamutexon();
    reset_state_and_idle();
#if DEBUG_CALLBACKS
    Serial.printf("[RX_ERR] #%lu Error occurred\n", rx_err_count);
#endif
    print_status_state("RX_ERR");
    decamutexoff(stat);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    tx_conf_count++;

    decaIrqStatus_t stat = decamutexon();

    if (current_state == STATE_IDLE)
    {
        current_state = STATE_POLL_SENT;
    }
    else
    {
        current_state = STATE_IDLE;
    }

    // only enable RX if we just sent a POLL frame
    if (current_state == STATE_POLL_SENT)
    {
        if (dwt_rxenable(DWT_START_RX_DELAYED) != DWT_SUCCESS)
        {
#if DEBUG_CALLBACKS
            Serial.println("[TX_CONF] ERROR: dwt_rxenable failed");
#endif
        }
        else
        {
#if DEBUG_CALLBACKS
            Serial.println("[TX_CONF] RX enabled after POLL sent");
#endif
        }
    }

#if DEBUG_CALLBACKS
    if (current_state == STATE_RESP_RECEIVED)
    {
        Serial.printf("[TX_CONF] #%lu FINAL sent, ranging #%lu complete\n", tx_conf_count, ranging_count);
    }
    else
    {
        Serial.printf("[TX_CONF] #%lu TX complete\n", tx_conf_count);
    }
#endif

    print_status_state("TX_CONF");

    decamutexoff(stat);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    dwt_readtxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    dwt_readrxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value.
 *        In the timestamp fields of the final message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8)ts;
        ts >>= 8;
    }
}
