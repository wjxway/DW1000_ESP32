/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions for ESP32-S3 using ESP-IDF SPI Master driver
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 * Modified for ESP32-S3 using ESP-IDF
 *
 * All rights reserved.
 */
#include <string.h>
#include <esp_rom_gpio.h>
#include <rom/ets_sys.h>

#include "deca_spi.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

static const char *TAG = "deca_spi";

/* Max SPI transfer size - DW1000 has max 1023 byte frame + header */
#define DW1000_SPI_MAX_TRANSFER_SIZE 1030

/* Fixed SPI speeds */
static const uint32_t SPI_SPEED_SLOW = 3000000;  // 3 MHz for initialization
static const uint32_t SPI_SPEED_FAST = 20000000; // 20 MHz for normal operation

/* Static TX/RX buffers for SPI transactions (DMA capable) */
static uint8_t WORD_ALIGNED_ATTR spi_tx_buffer[DW1000_SPI_MAX_TRANSFER_SIZE];
static uint8_t WORD_ALIGNED_ATTR spi_rx_buffer[DW1000_SPI_MAX_TRANSFER_SIZE];

/* Global variables */
static spi_device_handle_t dw1000_spi_handle = NULL;
static spi_host_device_t spi_peripheral;
static gpio_num_t cs_pin;
static spi_device_interface_config_t spi_dev_cfg;
static bool spi_bus_initialized_by_us = false;
static bool spi_auto_bus_aquisition = true;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_spi_init()
 *
 * @brief Initialize the SPI interface with the given configuration
 * WARNING: After calling dwt_initialise(), you must call dw1000_spi_fix_bug() before normal operation
 */
int dw1000_spi_init(spi_host_device_t spi_peripheral_in, gpio_num_t io_cs, const spi_bus_config_t *spi_bus)
{
    esp_err_t ret;

    /* Save configuration */
    spi_peripheral = spi_peripheral_in;
    cs_pin = io_cs;

    /* Configure GPIO for CS (manual control) - always needed */
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << io_cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&cs_conf);
    gpio_set_level(io_cs, 1); /* CS high (inactive) */

    /* Initialize SPI bus if bus config is provided */
    if (spi_bus != NULL)
    {
        /* Use provided bus configuration */
        ret = spi_bus_initialize(spi_peripheral, spi_bus, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            return -1;
        }
        if (ret == ESP_OK)
        {
            spi_bus_initialized_by_us = true;
        }
    }
    /* If spi_bus is NULL, assume bus is already initialized externally */

    /* Configure SPI device - start with slow speed */
    memset(&spi_dev_cfg, 0, sizeof(spi_dev_cfg));

    /* DW1000 Requirements: SPI Mode 0, MSB first, Max 20 MHz */
    spi_dev_cfg.mode = 0;
    spi_dev_cfg.clock_speed_hz = SPI_SPEED_SLOW;
    spi_dev_cfg.spics_io_num = -1; /* Manual CS control */
    spi_dev_cfg.queue_size = 5;
    spi_dev_cfg.flags = 0;

    /* Add device to bus */
    ret = spi_bus_add_device(spi_peripheral, &spi_dev_cfg, &dw1000_spi_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "DW1000 SPI initialized successfully");
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_spi_deinit()
 *
 * @brief De-initialize the SPI interface
 */
void dw1000_spi_deinit(void)
{
    if (dw1000_spi_handle != NULL)
    {
        spi_bus_remove_device(dw1000_spi_handle);
        dw1000_spi_handle = NULL;
    }

    if (spi_bus_initialized_by_us)
    {
        spi_bus_free(spi_peripheral);
        spi_bus_initialized_by_us = false;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to low speed for initialization
 */
void spi_set_rate_low(void)
{
    if (dw1000_spi_handle != NULL)
    {
        spi_bus_remove_device(dw1000_spi_handle);

        spi_dev_cfg.clock_speed_hz = SPI_SPEED_SLOW;
        spi_bus_add_device(spi_peripheral, &spi_dev_cfg, &dw1000_spi_handle);

        ESP_LOGI(TAG, "SPI rate set to low: %ld Hz", SPI_SPEED_SLOW);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate to high speed for normal operation
 */
void spi_set_rate_high(void)
{
    if (dw1000_spi_handle != NULL)
    {
        spi_bus_remove_device(dw1000_spi_handle);

        spi_dev_cfg.clock_speed_hz = SPI_SPEED_FAST;
        spi_bus_add_device(spi_peripheral, &spi_dev_cfg, &dw1000_spi_handle);

        ESP_LOGI(TAG, "SPI rate set to high: %ld Hz", (long)SPI_SPEED_FAST);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_spi_fix_bug()
 *
 * @brief Apply SPI bug fix workaround for DW1000
 *
 * This function must be called after dwt_initialise() and before normal operation.
 * It temporarily sets SPI to high speed, sets bit 10 in SYS_CFG register, then returns to low speed.
 *
 * WARNING: Call this after dwt_initialise() and before setting SPI to high speed for normal operation
 */
void dw1000_spi_fix_bug(void)
{
    /* Set SPI to high speed temporarily */
    spi_set_rate_high();

    /* Set bit 10 in SYS_CFG register */
    dwt_write32bitreg(SYS_CFG_ID, 0x1600);

    /* Return to low speed */
    spi_set_rate_low();

    ESP_LOGI(TAG, "DW1000 SPI bug fix applied");
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void)
{
    /* SPI initialization is done by dw1000_spi_init() */
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    /* SPI de-initialization is done by dw1000_spi_deinit() */
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    esp_err_t ret;
    spi_transaction_t trans;
    decaIrqStatus_t stat;

    if (dw1000_spi_handle == NULL)
    {
        ESP_LOGE(TAG, "SPI not initialized");
        return -1;
    }

    if (headerLength > DW1000_SPI_MAX_TRANSFER_SIZE || bodylength > DW1000_SPI_MAX_TRANSFER_SIZE)
    {
        ESP_LOGE(TAG, "Transfer size too large");
        return -1;
    }

    /* Disable DW1000 IRQ during SPI transaction */
    stat = decamutexon();

    /* Acquire SPI bus */
    if (spi_auto_bus_aquisition)
    {
        ret = dw1000_spi_acquire_bus();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to acquire SPI bus: %s", esp_err_to_name(ret));
            decamutexoff(stat);
            return -1;
        }
    }

    /* Assert CS */
    gpio_set_level(cs_pin, 0);

    /* Send header */
    memset(&trans, 0, sizeof(trans));
    trans.length = headerLength * 8;
    trans.tx_buffer = headerBuffer;
    trans.rx_buffer = NULL;
    ret = spi_device_polling_transmit(dw1000_spi_handle, &trans);

    /* Send body if present */
    if (ret == ESP_OK && bodylength > 0 && bodyBuffer != NULL)
    {
        memset(&trans, 0, sizeof(trans));
        trans.length = bodylength * 8;
        trans.tx_buffer = bodyBuffer;
        trans.rx_buffer = NULL;
        ret = spi_device_polling_transmit(dw1000_spi_handle, &trans);
    }

    /* De-assert CS */
    gpio_set_level(cs_pin, 1);

    /* Release SPI bus */
    if (spi_auto_bus_aquisition)
    {
        dw1000_spi_release_bus();
    }

    /* Re-enable DW1000 IRQ */
    decamutexoff(stat);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
        return -1;
    }

    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    esp_err_t ret;
    spi_transaction_t trans;
    decaIrqStatus_t stat;

    if (dw1000_spi_handle == NULL)
    {
        ESP_LOGE(TAG, "SPI not initialized");
        return -1;
    }

    if (headerLength > DW1000_SPI_MAX_TRANSFER_SIZE || readlength > DW1000_SPI_MAX_TRANSFER_SIZE)
    {
        ESP_LOGE(TAG, "Transfer size too large");
        return -1;
    }

    /* Disable DW1000 IRQ during SPI transaction */
    stat = decamutexon();

    /* Acquire SPI bus */
    if (spi_auto_bus_aquisition)
    {
        ret = dw1000_spi_acquire_bus();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to acquire SPI bus: %s", esp_err_to_name(ret));
            decamutexoff(stat);
            return -1;
        }
    }

    /* Assert CS */
    gpio_set_level(cs_pin, 0);

    /* Send header */
    memset(&trans, 0, sizeof(trans));
    trans.length = headerLength * 8;
    trans.tx_buffer = headerBuffer;
    trans.rx_buffer = NULL;
    ret = spi_device_polling_transmit(dw1000_spi_handle, &trans);

    /* Read data */
    if (ret == ESP_OK && readlength > 0)
    {
        memset(&trans, 0, sizeof(trans));
        trans.length = readlength * 8;
        trans.tx_buffer = NULL;
        trans.rx_buffer = readBuffer;
        ret = spi_device_polling_transmit(dw1000_spi_handle, &trans);
    }

    /* De-assert CS */
    gpio_set_level(cs_pin, 1);

    /* Release SPI bus */
    if (spi_auto_bus_aquisition)
    {
        dw1000_spi_release_bus();
    }

    /* Re-enable DW1000 IRQ */
    decamutexoff(stat);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
        return -1;
    }

    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_auto_bus_aquisition()
 *
 * Enable or disable automatic bus acquisition for DW1000, when enabled, for each SPI transaction, the driver will automatically
 * acquire the bus and release it after the transaction is completed.
 * You might want to disable this feature if you want to do multiple SPI transactions in a row without releasing the bus in between.
 * @param enable - true to enable, false to disable
 * @note This function is enabled by default.
 * @warning You should be very careful when calling this. When it happens to be called during a SPI transaction, it can either
 * cause a deadlock or throw an fatal error.
 */
int dw1000_auto_bus_acquisition(bool enable)
{
    spi_auto_bus_aquisition = enable;
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_spi_acquire_bus()
 *
 * Acquire the SPI bus, this function will block until the bus is available.
 * @return ESP_OK if the bus was acquired successfully, or an error code if there was an error.
 * @warning This function is called automatically by the driver when automatic bus acquisition is enabled.
 * @warning You should NEVER call this function if automatic bus acquisition is enabled.
 * @note This function will not consume any CPU time while waiting.
 */
esp_err_t dw1000_spi_acquire_bus()
{
    return spi_device_acquire_bus(dw1000_spi_handle, portMAX_DELAY);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_spi_release_bus()
 *
 * Release the SPI bus, this function will release the bus immediately.
 * @warning This function is called automatically by the driver when automatic bus acquisition is enabled.
 * @warning You should NEVER call this function if automatic bus acquisition is enabled. */
void dw1000_spi_release_bus()
{
    spi_device_release_bus(dw1000_spi_handle);
}