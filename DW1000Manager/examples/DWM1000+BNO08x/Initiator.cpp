/**
 * @file Initiator.cpp
 * @brief UWB DS-TWR Initiator Test
 *
 * This test initializes the DW1000 as an initiator and triggers
 * ranging exchanges every 20ms.
 */

#include <Arduino.h>
#include <DW1000Initiator.hpp>
#include <HardwareDefs.hpp>
#include <Blink.hpp>
#include <driver/spi_master.h>

void setup()
{
    Serial.begin(115200);
    vTaskDelay(1000);

    /* Blink LED to indicate start */
    Blink(500, 3, true, true, true);

    Serial.println("\n=== UWB DS-TWR Initiator Test ===");
    Serial.println("Initializing...");

    // this is only necessary on my test board where another IMU is present.
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);

    // Initialize SPI bus before initializing any other thing
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
        .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
        .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0};

    if (spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK)
    {
        Serial.println("ERROR: Failed to initialize SPI bus!");
        while (1)
        {
            vTaskDelay(1000);
        }
    }

    // Initialize the UWB hardware
    if (!UWBRanging::Initiator::Initialize(UWB_CS_PIN, UWB_IRQ_PIN, UWB_RST_PIN, 6, 2))
    {
        Serial.println("ERROR: Failed to initialize UWB Initiator!");
        while (1)
        {
            vTaskDelay(1000);
        }
    }

    Serial.println("UWB Initiator initialized successfully");

    // Start ranging with 100ms interval
    UWBRanging::Initiator::Begin(100);

    Serial.println("Initiator is now sending ranging polls...\n");
}

void loop()
{
    // Check if still active
    if (UWBRanging::Initiator::IsActive())
    {
        static uint32_t last_status_time = 0;
        uint32_t current_time = millis();

        // Print status every 5 seconds
        if (current_time - last_status_time >= 5000)
        {
            Serial.println("Initiator active and ranging...");
            last_status_time = current_time;
        }
    }
    else
    {
        Serial.println("WARNING: Initiator is not active!");
    }

    vTaskDelay(100);
}
