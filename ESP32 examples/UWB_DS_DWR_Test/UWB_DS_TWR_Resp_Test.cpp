/**
 * @file UWB_DS_TWR_Resp_Test.cpp
 * @brief Double-sided Two-Way Ranging (DS-TWR) Responder using interrupts for ESP32-S3
 *
 * This application acts as the responder in a DS-TWR distance measurement exchange.
 * It waits for a "poll" message from the initiator, sends a "response", then waits
 * for a "final" message containing all timestamps to compute the distance.
 * All RX/TX events are handled through ISR callbacks.
 * Based on ex_05b_ds_twr_resp rewritten to use interrupt-driven operation.
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
 * Poll message: initiator -> responder (trigger ranging)
 * Response message: responder -> initiator (continue ranging)
 * Final message: initiator -> responder (complete ranging with timestamps) */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received messages. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. */
/* This is the delay from Frame RX timestamp to TX reply timestamp. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
/* This is the delay from the end of the frame transmission to the enable of the receiver. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 900
/* Receive final timeout. */
#define FINAL_RX_TIMEOUT_UUS 2000

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here. */
static double tof;
static double distance;

/* State machine for DS-TWR responder */
typedef enum
{
    STATE_WAITING_POLL,
    STATE_POLL_RECEIVED,
    STATE_RESP_SENT
} ds_twr_resp_state_t;

static volatile ds_twr_resp_state_t current_state = STATE_WAITING_POLL;

/* reset state and go back to wait poll */
static void reset_state_and_wait_poll()
{
    current_state = STATE_WAITING_POLL;
    dwt_setrxtimeout(0);
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* Callback statistics */
static volatile uint32_t tx_conf_count = 0;
static volatile uint32_t rx_ok_count = 0;
static volatile uint32_t rx_to_count = 0;
static volatile uint32_t rx_err_count = 0;

/* Ranging count and last distance */
static volatile uint32_t ranging_count = 0;
static volatile double last_distance = 0.0;

/* Forward declarations of callback functions */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

/* Forward declarations of helper functions */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

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
    Serial.println("   DS-TWR Responder Test (Interrupts)");
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
    Serial.println("   Starting DS-TWR Responder...");
    Serial.println("========================================\n");

    reset_state_and_wait_poll();

    // disable auto-bus acquisition to allow manual control of the SPI bus locks
    }

/**
 * @brief Arduino loop function - monitors ranging status
 */
void loop()
{
    /* Print status periodically */
    Serial.printf("\n=== Stats: TX: %lu, RX OK: %lu, RX TO: %lu, RX Err: %lu | Ranging: %lu, Distance: %.3f m ===\n",
                  tx_conf_count, rx_ok_count, rx_to_count, rx_err_count, ranging_count, last_distance);
    print_status_state("LOOP");

    vTaskDelay(pdMS_TO_TICKS(1000));
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

    /* A frame has been received, copy it to our local buffer */
    if (cb_data->datalength <= RX_BUF_LEN)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);

        /* Clear sequence number for validation */
        rx_buffer[ALL_MSG_SN_IDX] = 0;

        /* Check what type of message we received based on current state */
        if (current_state == STATE_WAITING_POLL && memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            /* Poll message received */
            current_state = STATE_POLL_RECEIVED;

            /* Retrieve poll reception timestamp */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Set send time for response */
            uint32 resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Write and send the response message */
            tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* ranging bit set */

            int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

            if (ret == DWT_SUCCESS)
            {
                frame_seq_nb++;
                decamutexoff(stat);
                print_status_state("RX_OK");
#if DEBUG_CALLBACKS
                Serial.printf("[RX_OK] #%lu POLL received, RX ts: %llu, sending response\n", rx_ok_count, poll_rx_ts);
#endif
                return;
            }
            else
            {
                reset_state_and_wait_poll();
                decamutexoff(stat);
                print_status_state("RX_OK");
#if DEBUG_CALLBACKS
                Serial.printf("[RX_OK] #%lu ERROR: Response TX failed (late): %d\n", rx_ok_count, ret);
#endif
                return;
            }
        }
        else if ((current_state == STATE_RESP_SENT) && memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            /* Final message received */
            uint32 poll_tx_ts_32, resp_rx_ts_32, final_tx_ts_32;
            uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
            double Ra, Rb, Da, Db;
            int64 tof_dtu;

            /* Retrieve response transmission and final reception timestamps */
            resp_tx_ts = get_tx_timestamp_u64();
            final_rx_ts = get_rx_timestamp_u64();

            /* Get timestamps embedded in the final message */
            final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts_32);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts_32);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_32);

            /* Compute time of flight using 32-bit subtractions (handles clock wrap) */
            poll_rx_ts_32 = (uint32)poll_rx_ts;
            resp_tx_ts_32 = (uint32)resp_tx_ts;
            final_rx_ts_32 = (uint32)final_rx_ts;

            Ra = (double)(resp_rx_ts_32 - poll_tx_ts_32);
            Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
            Da = (double)(final_tx_ts_32 - resp_rx_ts_32);
            Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);

            tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

            tof = tof_dtu * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;
            last_distance = distance;
            ranging_count++;

            /* Re-enable reception for next poll */
            reset_state_and_wait_poll();
            decamutexoff(stat);
            print_status_state("RX_OK");
#if DEBUG_CALLBACKS
            Serial.printf("[RX_OK] #%lu FINAL received, distance: %.3f m (ToF: %.3f ns)\n", rx_ok_count, distance, tof * 1e9);
#endif
            return;
        }
        else
        {
            reset_state_and_wait_poll();
            decamutexoff(stat);
            print_status_state("RX_OK");
#if DEBUG_CALLBACKS
            Serial.printf("[RX_OK] #%lu Unexpected frame in state %d, restarting\n", rx_ok_count, current_state);
#endif
            return;
        }
    }
    else
    {
        reset_state_and_wait_poll();
        decamutexoff(stat);
        print_status_state("RX_OK");
#if DEBUG_CALLBACKS
        Serial.printf("[RX_OK] #%lu Frame too long (%u > %d)\n", rx_ok_count, cb_data->datalength, RX_BUF_LEN);
#endif
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
    reset_state_and_wait_poll();
    decamutexoff(stat);

    print_status_state("RX_TO");
#if DEBUG_CALLBACKS
    Serial.printf("[RX_TO] #%lu Timeout occurred\n", rx_to_count);
#endif
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
    reset_state_and_wait_poll();
    decamutexoff(stat);

    print_status_state("RX_ERR");
#if DEBUG_CALLBACKS
    Serial.printf("[RX_ERR] #%lu Error occurred\n", rx_err_count);
#endif
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
    current_state = STATE_RESP_SENT;

    decaIrqStatus_t stat = decamutexon();
    dwt_forcetrxoff();
    /* Set expected delay and timeout for final message reception */
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
    dwt_rxenable(DWT_START_RX_DELAYED);
    decamutexoff(stat);

    print_status_state("TX_CONF");
#if DEBUG_CALLBACKS
    Serial.printf("[TX_CONF] #%lu Response sent, waiting for final message\n", tx_conf_count);
#endif
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
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message.
 *        In the timestamp fields of the final message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    *ts = 0;
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
