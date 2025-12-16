/**
 * @file UWB_RX_Send_Test.cpp
 * @brief Receive frame and send response test using interrupts for ESP32-S3
 *
 * This application waits to receive a UWB frame using interrupts, validates it,
 * and sends a response. All RX/TX events are handled through ISR callbacks.
 * Based on ex_03b_rx_send_resp rewritten to use interrupt-driven operation.
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
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
}

/* Debug macros */
#define DEBUG_STATUS_STATE 0 // Enable/disable status and state register printing
#define DEBUG_CALLBACKS 1    // Enable/disable callback debug prints

/* Configuration for optimal operation - same as UWB_TX_Test.cpp */
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

/* Response message to send back when a valid frame is received.
 * Format: {0xC5, sequence_number, 0xFE, 0xDC, 0xBA, 0x98, 0xDE, 0xCA, 0x00, 0x00}
 * The sequence number will be copied from the received frame. */
static uint8 tx_msg[] = {0xC5, 0x00, 0xDE, 0xCA, 0x98, 0x76, 0x54, 0x32, 0x10, 0x00, 0x00};
/* Index to access to sequence number in the tx_msg array. */
#define RESP_FRAME_SN_IDX 1
#define BLINK_FRAME_SN_IDX 1

/* Delay from end of transmission to activation of reception, expressed in us */
#define RX_TO_TX_DELAY_US 1000

/* Buffer to store received frame. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Callback statistics */
static volatile uint32_t rx_ok_count = 0;
static volatile uint32_t rx_err_count = 0;
static volatile uint32_t tx_conf_count = 0;

/* Response count */
static volatile uint32_t resp_count = 0;

/* Forward declarations of callback functions */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

static void print_status_state(const char *prefix)
{
#if DEBUG_STATUS_STATE

    sys_status_reg_t status;
    sys_state_reg_t state;
    char status_buf[512], state_buf[512];
    deca_get_sys_status(&status);
    deca_get_sys_state(&state);

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
    delay(1000);

    /* Blink LED to indicate start */
    Blink(500, 3, true, true, true);

    Serial.println("\n========================================\n"
                   "   RX Send Response Test\n"
                   "========================================\n");

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
    delay(5);

    /* Initialize DW1000 (no microcode needed for RX-only initially) */
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
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

    /* Setup interrupt handling with callbacks using dw1000_setup_isr() */
    Serial.println("Setting up ISR with callbacks...");
    if (dw1000_setup_isr(6, &tx_conf_cb, &rx_ok_cb, NULL, &rx_err_cb) != 0)
    {
        Serial.println("ERROR: dw1000_setup_isr failed!");
        while (1)
            ;
    }
    Serial.println("ISR and callbacks configured (priority 6)");

    /* Enable wanted interrupts (TX confirmation, RX good frames, and RX errors) */
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
    Serial.println("Interrupts enabled: TFRS, RFCG, RPHE, RFCE, RFSL, SFDT");

    Serial.println("\n========================================\n"
                   "   Initialization Complete!\n"
                   "   Starting RX/Response loop...\n"
                   "========================================\n");

    int rx_ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    if (rx_ret != DWT_SUCCESS)
        Serial.printf("ERROR: dwt_rxenable failed: %d\n", rx_ret);

    // auto-bus acquisition enabled by default
}

/**
 * @brief Arduino loop function - waits for frames and sends responses using interrupts
 */
void loop()
{
    /* Print callback statistics */
    Serial.printf("\n=== Stats: RX OK: %lu, RX Err: %lu, TX: %lu | Responses: %lu ===\n",
                  rx_ok_count, rx_err_count, tx_conf_count, resp_count);
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

    /* A frame has been received, copy it to our local buffer */
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);

        /* Validate the frame (expected pattern: DE CA 01 23 45 67 89 starting from byte 2) */
        bool is_valid = (cb_data->datalength >= 9) &&
                        (rx_buffer[BLINK_FRAME_SN_IDX + 1] == 0xDE) &&
                        (rx_buffer[BLINK_FRAME_SN_IDX + 2] == 0xCA) &&
                        (rx_buffer[BLINK_FRAME_SN_IDX + 3] == 0x01) &&
                        (rx_buffer[BLINK_FRAME_SN_IDX + 4] == 0x23) &&
                        (rx_buffer[BLINK_FRAME_SN_IDX + 5] == 0x45) &&
                        (rx_buffer[BLINK_FRAME_SN_IDX + 6] == 0x67) &&
                        (rx_buffer[BLINK_FRAME_SN_IDX + 7] == 0x89);

        if (is_valid)
        {
            /* Copy sequence number from received frame to response */
            tx_msg[RESP_FRAME_SN_IDX] = rx_buffer[BLINK_FRAME_SN_IDX];

            /* Write response frame data to DW1000 and prepare transmission */
            dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
            dwt_writetxfctrl(sizeof(tx_msg), 0, 0);

            // Wait for some time before sending the response.
            // in reality, you probably would want to enable micro code through
            //      dwt_initialise(DWT_LOADUCODE);
            // and use internal timer to schedule the response
            //      #define UUS_TO_DWT_TIME 65536
            //      resp_rx_ts = get_rx_timestamp_u64();
            //      uint32 final_tx_time = (resp_rx_ts + (RX_TO_TX_DELAY_UUS * UUS_TO_DWT_TIME)) >> 8;
            //      dwt_setdelayedtrxtime(final_tx_time);
            //      dwt_starttx(DWT_START_TX_DELAYED);
            // for more details, please refer to DS_TWR examples
            delayMicroseconds(RX_TO_TX_DELAY_US);

            /* Send the response */
            int tx_ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

            print_status_state("RX_OK");
#if DEBUG_CALLBACKS
            Serial.printf("[RX_OK] #%lu Valid frame received, seq %u, sending response\n",
                          rx_ok_count, rx_buffer[BLINK_FRAME_SN_IDX]);
#endif
            if (tx_ret != DWT_SUCCESS)
            {
#if DEBUG_CALLBACKS
                Serial.printf("[RX_OK] ERROR: dwt_starttx failed: %d\n", tx_ret);
#endif
            }
        }
        else
        {
            print_status_state("RX_OK");
#if DEBUG_CALLBACKS
            Serial.printf("[RX_OK] #%lu Invalid frame, ignoring\n", rx_ok_count);
#endif
        }
    }
    else
    {
        print_status_state("RX_OK");
#if DEBUG_CALLBACKS
        Serial.printf("[RX_OK] #%lu Frame too long (%u > %d)\n", rx_ok_count, cb_data->datalength, FRAME_LEN_MAX);
#endif
    }
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

    /* Restart reception after an error */
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

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

    /* Activate reception immediately after send complete */
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    print_status_state("TX_CONF");
#if DEBUG_CALLBACKS
    Serial.printf("[TX_CONF] #%lu Response sent, RX re-enabled\n", tx_conf_count);
#endif
}
