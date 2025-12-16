/**
 * @file UWB_RX_Test.cpp
 * @brief Simple UWB receiver test for ESP32-S3
 *
 * This application continuously receives UWB frames using the DW1000 transceiver.
 * It polls for incoming frames, validates reception, and prints received data to serial.
 * Based on ex_02a_simple_rx with polling-based frame reception.
 */

#include <Arduino.h>
#include <HardwareDefs.hpp>
#include <Blink.hpp>
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

/* Default antenna delay values for 64 MHz PRF */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

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

    Serial.println("\n========================================");
    Serial.println("             RX reader");
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
    delay(5);

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

    /* Apply default antenna delay value */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    Serial.println("\n========================================");
    Serial.println("   Initialization Complete!");
    Serial.println("   Waiting for messages...");
    Serial.println("========================================\n");
}

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

static uint32_t status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

/**
 * @brief Arduino loop function - performs SS TWR ranging response
 */
void loop()
{
    /* TESTING BREAKPOINT LOCATION #1 */

    /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
     * the RX buffer.
     * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
     * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
    for (int i = 0; i < FRAME_LEN_MAX; i++)
    {
        rx_buffer[i] = 0;
    }

    /* Activate reception immediately. See NOTE 3 below. */
    int rx_enable_ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    if (rx_enable_ret != DWT_SUCCESS)
    {
        Serial.printf("ERROR: dwt_rxenable failed with code %d\n", rx_enable_ret);
    }

    /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 4 below.
     * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
     * function to access it. */
    int64_t t_start = esp_timer_get_time();
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
    {
        if (esp_timer_get_time() - t_start > 2000000) // 2 second timeout
        {
            Serial.println("RX Timeout");
            t_start = esp_timer_get_time();
        }
        vTaskDelay(1);
    };

    /* Print RX event state before clearing or reading */
    Serial.printf("RX Event - Status register: 0x%08lX\n", status_reg);

    if (status_reg & SYS_STATUS_RXFCG)
    {
        /* A frame has been received, copy it to our local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        Serial.printf("Frame received - Length: %d bytes\n", frame_len);

        if (frame_len <= FRAME_LEN_MAX)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            Serial.print("RX Data: ");
            for (int i = 0; i < frame_len; i++)
            {
                Serial.printf("%02X ", rx_buffer[i]);
            }
            Serial.println();
        }
        else
        {
            Serial.printf("ERROR: Frame too long (%d > %d)\n", frame_len, FRAME_LEN_MAX);
        }

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    }
    else
    {
        Serial.println("RX Error occurred");
        /* Clear RX error events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        Serial.println("RX error events cleared");
    }

    // give it a break...
    vTaskDelay(1);
}