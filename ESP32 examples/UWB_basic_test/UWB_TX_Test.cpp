/**
 * @file UWB_TX_Test.cpp
 * @brief Simple UWB transmitter test for ESP32-S3
 *
 * This application continuously transmits UWB blink frames using the DW1000 transceiver.
 * It sends 802.15.4e standard blink frames with incrementing sequence numbers and
 * polls the status register to confirm successful transmission.
 * Based on ex_01a_simple_tx with polling-based transmission confirmation.
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

// /* Configuration for maximum range and reliability operation */
// static dwt_config_t dw1000_config = {
//     1,               /* Channel number. */
//     DWT_PRF_64M,     /* Pulse repetition frequency. */
//     DWT_PLEN_4096,   /* Preamble length. Used in TX only. */
//     DWT_PAC64,       /* Preamble acquisition chunk size. Used in RX only. */
//     9,               /* TX preamble code. Used in TX only. */
//     9,               /* RX preamble code. Used in RX only. */
//     1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
//     DWT_BR_110K,     /* Data rate. */
//     DWT_PHRMODE_STD, /* PHY header mode. */
//     (4096 + 16 + 64 - 64) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
// };

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
    Serial.println("              TX sender");
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
    Serial.println("SPI initialized");

    /* Configure DW1000 GPIO */
    if (dw1000_gpio_init((gpio_num_t)UWB_RST_PIN, (gpio_num_t)UWB_IRQ_PIN, GPIO_NUM_NC) != 0)
    {
        Serial.println("ERROR: GPIO init failed!");
        while (1)
            ;
    }
    Serial.println("GPIO initialized");

    /* Reset DW1000 */
    dw1000_hard_reset();
    dw1000_spi_fix_bug();
    Serial.println("DW1000 reset complete");
    vTaskDelay(5);

    /* Initialize DW1000 with microcode */
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        Serial.println("ERROR: dwt_initialise failed!");
        while (1)
            ;
    }
    Serial.println("DW1000 initialized");

    /* Set SPI to high speed */
    spi_set_rate_high();

    /* Configure DW1000 */
    dwt_configure(&dw1000_config);
    Serial.println("DW1000 configured");

    // /* Apply default antenna delay value */
    // dwt_setrxantennadelay(RX_ANT_DLY);
    // dwt_settxantennadelay(TX_ANT_DLY);

    Serial.println("\n========================================");
    Serial.println("   Initialization Complete!");
    Serial.println("   Starting TX...");
    Serial.println("========================================\n");
}

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 8: message content.
 *     - byte 9/10: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0x00, 0xDE, 0xCA, 0x01, 0x23, 0x45, 0x67, 0x89, 0x00, 0x00};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 200

/**
 * @brief Arduino loop function - performs SS TWR ranging
 */
void loop()
{
    /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
    int write_ret = dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
    if (write_ret != DWT_SUCCESS)
    {
        Serial.printf("ERROR: dwt_writetxdata failed with code %d\n", write_ret);
    }

    dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

    /* Start transmission. */
    int tx_ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
    if (tx_ret != DWT_SUCCESS)
    {
        Serial.printf("ERROR: dwt_starttx failed with code %d\n", tx_ret);
    }

    /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
     * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
     * function to access it.*/
    uint32_t status;

    /* Print TX event state before polling */
    status = dwt_read32bitreg(SYS_STATUS_ID);
    Serial.printf("TX started - Status register: 0x%08lX\n", status);

    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
    {
    };

    /* Print TX event state before clearing */
    Serial.printf("TX event detected - status register: 0x%08lX\n", status);

    /* Clear TX frame sent event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    /* Print TX event state after clearing */
    status = dwt_read32bitreg(SYS_STATUS_ID);
    Serial.printf("TX event cleared - status register: 0x%08lX\n\n", status);

    /* Execute a delay between transmissions. */
    vTaskDelay(pdMS_TO_TICKS(TX_DELAY_MS));

    /* Increment the blink frame sequence number (modulo 256). */
    tx_msg[BLINK_FRAME_SN_IDX]++;
}