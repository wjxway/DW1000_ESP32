/**
 * @file NewRollbot_IMU_Test.cpp
 * @brief Test program for IMU functionality
 *
 * This test initializes the BNO08x IMU and prints sensor data
 * every 500ms to verify proper operation.
 */

#include <Arduino.h>
#include <HardwareDefs.hpp>
#include <Blink.hpp>

#include <driver/spi_common.h>
#include <driver/spi_master.h>

spi_device_handle_t spi_dev_1, spi_dev_2;

spi_transaction_t spi_transaction;

uint8_t tx_buffer_1[100], tx_buffer_2[100];

void try_spi_task(void *pvParameters)
{
    while (1)
    {
        // Your task implementation here
        spi_device_acquire_bus(spi_dev_2, portMAX_DELAY); // acquire the SPI bus

        Serial.println("try_spi_task acquired spi_dev_2");
        gpio_set_level((gpio_num_t)UWB_CS_PIN, 0);                // assert chip select
        spi_device_polling_transmit(spi_dev_2, &spi_transaction); // send data packet
        gpio_set_level((gpio_num_t)UWB_CS_PIN, 1);                // de-assert chip select

        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for 100ms

        // Serial.println("try_spi_task acquiring spi_dev_2 twice?"); // it will indefinitely poll.
        // spi_device_acquire_bus(spi_dev_2, portMAX_DELAY); // acquire the SPI bus

        // Serial.println("what if we released it twice?"); // it will call a failed assert!
        // spi_device_release_bus(spi_dev_2);
        
        Serial.println("try_spi_task releasing spi_dev_2");
        spi_device_release_bus(spi_dev_2); // release the SPI bus
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for 10ms
    }
}

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    vTaskDelay(1000);
    Blink(500, 3);

    spi_device_interface_config_t spi_dev_config;
    spi_bus_config_t bus_config;

    spi_dev_config.mode = 0x3; // set mode to 3 as per BNO08x datasheet (CPHA second edge, CPOL bus high when idle)

    spi_dev_config.clock_speed_hz = 1000000; // assign SCLK speed
    spi_dev_config.address_bits = 0;         // 0 address bits, not using this system
    spi_dev_config.command_bits = 0;         // 0 command bits, not using this system
    spi_dev_config.spics_io_num = -1;        // due to esp32 silicon issue, chip select cannot be used with full-duplex mode
    // driver, it must be handled via calls to gpio pins
    spi_dev_config.queue_size = 5; // only allow for 5 queued transactions at a timed

    bus_config.mosi_io_num = SPI_MOSI_PIN; // assign mosi gpio pin
    bus_config.miso_io_num = SPI_MISO_PIN; // assign miso gpio pin
    bus_config.sclk_io_num = SPI_CLK_PIN;  // assign sclk gpio pin
    bus_config.quadhd_io_num = -1;         // hold signal gpio (not used)
    bus_config.quadwp_io_num = -1;         // write protect signal gpio (not used)

    // do first SPI operation into nowhere before BNO085 reset to let periphiral stabilize (Anton B.)
    spi_transaction.length = 8;
    spi_transaction.rxlength = 0;
    spi_transaction.tx_buffer = tx_buffer_1;
    spi_transaction.rx_buffer = NULL;
    spi_transaction.flags = 0;

    // initialize the spi peripheral
    // Note: If the bus is already initialized (e.g., by DWM1000), this will return ESP_ERR_INVALID_STATE which is OK
    esp_err_t spi_ret = spi_bus_initialize(SPI2_HOST, &(bus_config), SPI_DMA_CH_AUTO);
    if (spi_ret != ESP_OK && spi_ret != ESP_ERR_INVALID_STATE)
    {
        Serial.println("Failed to initialize SPI bus");
    }

    // add the imu device to the bus
    spi_bus_add_device(SPI2_HOST, &(spi_dev_config), &(spi_dev_1));
    // add the uwb device to the bus
    spi_bus_add_device(SPI2_HOST, &(spi_dev_config), &(spi_dev_2));

    spi_device_acquire_bus(spi_dev_1, portMAX_DELAY);         // acquire the SPI bus
    gpio_set_level((gpio_num_t)IMU_CS_PIN, 0);                // assert chip select
    spi_device_polling_transmit(spi_dev_1, &spi_transaction); // send data packet
    gpio_set_level((gpio_num_t)IMU_CS_PIN, 1);                // de-assert chip select
    spi_device_release_bus(spi_dev_1);                        // release the SPI bus
    Serial.println("initial transmission on spi_dev_1 successful!");

    spi_device_acquire_bus(spi_dev_1, portMAX_DELAY); // acquire the SPI bus
    Serial.println("spi_dev_1 acquired spi bus");

    // try to create a task to use spi_dev_1
    xTaskCreate(&try_spi_task, "try_spi_task", 4096, NULL, 4, nullptr); // launch SPI task (data processing merged in)

    // Wait a bit
    vTaskDelay(100);
}

void loop()
{
    Serial.println("loop releasing spi_dev_1");
    spi_device_release_bus(spi_dev_1);                        // release the SPI bus

    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for 5ms

    // try to acquire the spi bus again
    spi_device_acquire_bus(spi_dev_1, portMAX_DELAY); // acquire the SPI bus
    Serial.println("loop acquired spi_dev_1");

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1000ms
}
