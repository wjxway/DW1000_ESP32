/*! ----------------------------------------------------------------------------
 * @file    deca_gpio.h
 * @brief   GPIO control functions for DW1000
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 * Modified for ESP32-S3 using ESP-IDF
 *
 * All rights reserved.
 */

#ifndef _DECA_GPIO_H_
#define _DECA_GPIO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <driver/gpio.h>
#include "decadriver/deca_device_api.h"

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_gpio_init()
     *
     * Initialize GPIO pins for DW1000 (RST, IRQ, WAKE)
     * Simple pin configuration only, does not setup interrupts
     * @param io_rst - Reset pin
     * @param io_irq - IRQ pin
     * @param io_wake - Wake pin (set to -1 if not used)
     * returns 0 for success, or -1 for error
     */
    int dw1000_gpio_init(gpio_num_t io_rst, gpio_num_t io_irq, gpio_num_t io_wake);

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_gpio_deinit()
     *
     * De-initialize GPIO pins and cleanup ISR/task if configured
     */
    void dw1000_gpio_deinit(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_setup_isr()
     *
     * Setup interrupt handling for DW1000
     * Creates FreeRTOS task, registers ISR handler, and configures dwt callbacks
     * ISR notifies task, task calls dwt_isr() which dispatches to the configured callbacks
     *
     * @param task_priority - Priority for IRQ handling task (default: 6 if set to 0)
     * @param cbTxDone - TX done callback (can be NULL)
     * @param cbRxOk - RX success callback (can be NULL)
     * @param cbRxTo - RX timeout callback (can be NULL)
     * @param cbRxErr - RX error callback (can be NULL)
     * returns 0 for success, or -1 for error
     */
    int dw1000_setup_isr(uint32_t task_priority,
                         dwt_cb_t cbTxDone,
                         dwt_cb_t cbRxOk,
                         dwt_cb_t cbRxTo,
                         dwt_cb_t cbRxErr);

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_unregister_isr()
     *
     * Unregister interrupt handling for DW1000
     * Disables interrupt, removes ISR handler, and deletes task
     */
    void dw1000_unregister_isr(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_hard_reset()
     *
     * Hardware reset of DW1000 using RST pin
     *
     * @warning After reset and dwt_initialise(), you must call dw1000_spi_fix_bug() before normal operation
     */
    void dw1000_hard_reset(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_wake_up()
     *
     * Pull wake up and hold 1ms then release to wake up DW1000
     */
    void dw1000_wake_up(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_gpio_enable_irq()
     *
     * Enable DW1000 IRQ interrupt
     */
    void dw1000_gpio_enable_irq(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_gpio_disable_irq()
     *
     * Disable DW1000 IRQ interrupt
     */
    void dw1000_gpio_disable_irq(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * Function: dw1000_gpio_get_irq_status()
     *
     * Get DW1000 IRQ enable status
     * returns 1 if enabled, 0 if disabled
     */
    bool dw1000_gpio_get_irq_status(void);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_GPIO_H_ */
