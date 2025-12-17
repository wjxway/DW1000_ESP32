/**
 * @file UWB_TX_Resp_Test.cpp
 * @brief Transmit and wait for response using interrupts for ESP32-S3
 *
 * This application sends a blink frame and waits for a response using interrupt callbacks.
 * It demonstrates full duplex communication with TX confirmation, RX reception, and error
 * handling all managed through ISR callbacks. The main loop processes events signaled
 * by the interrupt handlers.
 * Based on ex_03d_tx_wait_resp_interrupts with extensive status printing.
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

/* State machine */
typedef enum
{
    STATE_IDLE,
    STATE_TX_SENT,
    STATE_RESP_RECEIVED
} comm_state_t;

static volatile comm_state_t current_state = STATE_IDLE;

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 8: message content.
 *     - byte 9/10: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0x00, 0xDE, 0xCA, 0x01, 0x23, 0x45, 0x67, 0x89, 0x00, 0x00};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). */
#define TX_TO_RX_DELAY_UUS 500

/* Receive response timeout, expressed in UWB microseconds. */
#define RX_RESP_TO_UUS 5000

/* Buffer to store received frame. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Callback statistics */
static volatile uint32_t tx_conf_count = 0;
static volatile uint32_t rx_ok_count = 0;
static volatile uint32_t rx_to_count = 0;
static volatile uint32_t rx_err_count = 0;

/* TX count */
static volatile uint32_t tx_count = 0;

/* Forward declarations of callback functions */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
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
    vTaskDelay(1000);

    /* Blink LED to indicate start */
    Blink(500, 3, true, true, true);

    Serial.println("\n========================================\n"
                   "   TX Wait Response (Interrupts) Test\n"
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
    vTaskDelay(5);

    /* Initialize DW1000 (no microcode needed) */
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
    if (dw1000_setup_isr(6, &tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb) != 0)
    {
        Serial.println("ERROR: dw1000_setup_isr failed!");
        while (1)
            ;
    }
    Serial.println("ISR and callbacks configured (priority 6)");

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors) */
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_LDED | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE |
                         DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXOVRR | DWT_INT_RXPTO | DWT_INT_SFDT | DWT_INT_ARFE,
                     1);
    Serial.println("Interrupts enabled: TFRS, LDED, RFCG, RPHE, RFCE, RFSL, RFTO, RXOVRR, RXPTO, SFDT");

    /* Set delay to turn reception on after transmission of the frame */
    dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);
    dwt_setrxtimeout(RX_RESP_TO_UUS);
    Serial.printf("RX after TX delay: %u uus, RX timeout: %u uus\n", TX_TO_RX_DELAY_UUS, RX_RESP_TO_UUS);

    Serial.println("\n========================================\n"
                   "   Initialization Complete!\n"
                   "   Starting TX/RX loop...\n"
                   "========================================\n");

    // auto-bus acquisition enabled by default
}

/**
 * @brief Arduino loop function - sends frames periodically and waits for responses
 */
void loop()
{
    /* Print callback statistics */
    Serial.printf("\n=== Stats: TX: %lu, RX OK: %lu, RX TO: %lu, RX Err: %lu | TX Count: %lu ===\n",
                  tx_conf_count, rx_ok_count, rx_to_count, rx_err_count, tx_count);
    print_status_state("LOOP");

    /* Reset state */
    current_state = STATE_IDLE;

    /* Write frame data to DW1000 and prepare transmission */

    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
    dwt_writetxfctrl(sizeof(tx_msg), 0, 0);

    /* Start transmission with response expected */
    int tx_ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    if (tx_ret == DWT_SUCCESS)
    {
        tx_count++;
#if DEBUG_CALLBACKS
        Serial.printf("[LOOP] Sent frame #%lu, seq %u\n", tx_count, tx_msg[BLINK_FRAME_SN_IDX]);
#endif
    }
    else
    {
#if DEBUG_CALLBACKS
        Serial.printf("[LOOP] ERROR: dwt_starttx failed: %d\n", tx_ret);
#endif
    }

    /* Increment the blink frame sequence number (modulo 256) */
    tx_msg[BLINK_FRAME_SN_IDX]++;

    /* Wait 1 second before next transmission */
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
        current_state = STATE_RESP_RECEIVED;

        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);

        print_status_state("RX_OK");

        Serial.printf("[RX_OK] #%lu Frame received, length: %u bytes, data: ",
                      rx_ok_count, cb_data->datalength);
        for (int i = 0; i < cb_data->datalength; i++)
            Serial.printf("%02X ", rx_buffer[i]);
        Serial.println();
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
    current_state = STATE_IDLE;

    dwt_forcetrxoff();

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
    current_state = STATE_IDLE;

    dwt_forcetrxoff();

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

    /* Reset transceiver after TX confirmation */
    if (current_state == STATE_IDLE)
    {
        current_state = STATE_TX_SENT;

        dwt_forcetrxoff();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    print_status_state("TX_CONF");
#if DEBUG_CALLBACKS
    Serial.printf("[TX_CONF] #%lu TX complete, RX auto-activated\n", tx_conf_count);
#endif
}
