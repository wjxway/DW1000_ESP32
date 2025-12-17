/**
 * @file UWB_Init_Test.cpp
 * @brief DW1000 initialization test for ESP32-S3
 * 
 * This application tests the basic initialization sequence of the DW1000 UWB transceiver.
 * It configures SPI, GPIO, resets the device, loads microcode, and verifies the device ID.
 * Used to verify hardware connections and basic DW1000 functionality.
 */

#include <Arduino.h>
#include <HardwareDefs.hpp>
#include <Blink.hpp>

// Include DW1000 driver
extern "C"
{
#include "platform/deca_spi.h"
#include "platform/deca_gpio.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
}

/* DW1000 configuration - Channel 5, 850kbps, 16MHz PRF */
static dwt_config_t dw1000_config = {
    5,                /* Channel number. */
    DWT_PRF_16M,      /* Pulse repetition frequency. */
    DWT_PLEN_256,     /* Preamble length. Used in TX only. */
    DWT_PAC16,        /* Preamble acquisition chunk size. Used in RX only. */
    3,                /* TX preamble code. Used in TX only. */
    3,                /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_850K,      /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (256 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

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
    Serial.println("   DW1000 Simplified Init Test");
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
    dw1000_spi_fix_bug(); // Apply SPI bug fix after reset
    Serial.println("DW1000 reset complete");
    vTaskDelay(5);

    /* Initialize DW1000 with microcode */
    Serial.println("Calling dwt_initialise(DWT_LOADUCODE)...");
    int init_result = dwt_initialise(DWT_LOADUCODE);
    if (init_result == DWT_ERROR)
    {
        Serial.println("ERROR: dwt_initialise failed!");
        Serial.printf("Init result: %d\n", init_result);
        while (1)
            ;
    }
    Serial.println("SUCCESS: dwt_initialise passed");

    /* Set SPI to high speed */
    spi_set_rate_high();
    Serial.println("SPI speed set to high (16 MHz)");

    /* Configure DW1000 */
    dwt_configure(&dw1000_config);
    Serial.println("DW1000 configured");

    /* Read and print device ID */
    uint32_t dev_id = dwt_readdevid();
    Serial.printf("\nDevice ID: 0x%08lX\n", dev_id);

    Serial.println("\n========================================");
    Serial.println("   Initialization Complete!");
    Serial.println("========================================\n");
}

/**
 * @brief Arduino loop function
 */
void loop()
{
    vTaskDelay(1000);
}
