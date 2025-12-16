/*! ----------------------------------------------------------------------------
 * @file    deca_spi.h
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 * Modified for ESP32-S3 using ESP-IDF
 *
 * All rights reserved.
 */

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "decadriver/deca_types.h"

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_spi_init()
 *
 * Initialize the SPI interface and CS pin for DW1000
 * WARNING: After calling dwt_initialise(), you must call dw1000_spi_fix_bug() before normal operation
 * @param spi_peripheral - SPI host (SPI2_HOST or SPI3_HOST)
 * @param io_cs - CS pin number
 * @param spi_bus - Pointer to spi_bus_config_t, or NULL if bus is already initialized
 * returns 0 for success, or -1 for error
 */
int dw1000_spi_init(spi_host_device_t spi_peripheral, gpio_num_t io_cs, const spi_bus_config_t *spi_bus);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_spi_deinit()
 *
 * De-initialize the SPI interface
 */
void dw1000_spi_deinit(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: spi_set_rate_low()
 *
 * Set SPI rate to low speed for initialization
 */
void spi_set_rate_low(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: spi_set_rate_high()
 *
 * Set SPI rate to high speed for normal operation
 */
void spi_set_rate_high(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_spi_fix_bug()
 *
 * Apply SPI bug fix workaround for DW1000
 * @warning This function must be called before dwt_initialise()
 * It temporarily sets SPI to high speed, sets bit 10 in SYS_CFG register, then returns to low speed
 */
void dw1000_spi_fix_bug(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_auto_bus_aquisition()
 *
 * Enable or disable automatic bus acquisition for DW1000, when enabled, for each SPI transaction, the driver will automatically
 * acquire the bus and release it after the transaction is completed.
 * You might want to disable this feature if you want to do multiple SPI transactions in a row without releasing the bus in between.
 * @param enable - true to enable, false to disable
 * @note This function is enabled by default.
 */
int dw1000_auto_bus_acquisition(bool enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_spi_acquire_bus()
 *
 * Acquire the SPI bus, this function will block until the bus is available.
 * @return ESP_OK if the bus was acquired successfully, or an error code if there was an error.
 * @warning This function is called automatically by the driver when automatic bus acquisition is enabled.
 * @warning You should NEVER call this function if automatic bus acquisition is enabled.
 * @note This function will not consume any CPU time while waiting.
 */
esp_err_t dw1000_spi_acquire_bus();

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_spi_release_bus()
 *
 * Release the SPI bus, this function will release the bus immediately.
 * @warning This function is called automatically by the driver when automatic bus acquisition is enabled.
 * @warning You should NEVER call this function if automatic bus acquisition is enabled.
 */
void dw1000_spi_release_bus();

#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */
