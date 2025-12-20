/*! ----------------------------------------------------------------------------
 * @file    deca_gpio.c
 * @brief   GPIO control functions for DW1000
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 * Modified for ESP32-S3 using ESP-IDF
 *
 * All rights reserved.
 */

#include "deca_gpio.h"
#include "deca_spi.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char *TAG = "deca_gpio";

/* Global GPIO pin configuration */
static gpio_num_t gpio_pin_rst = GPIO_NUM_NC;
static gpio_num_t gpio_pin_irq = GPIO_NUM_NC;
static gpio_num_t gpio_pin_wake = GPIO_NUM_NC;
static volatile bool dw1000_irq_enabled = false;

/* FreeRTOS task handle and notification */
static TaskHandle_t dw1000_irq_task_handle = NULL;

/* IRQ task that calls dwt_isr() */
static void dw1000_irq_task(void *arg)
{
    while (1)
    {
        /* Wait for notification from ISR */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Call DW1000 ISR handler */
        decaIrqStatus_t stat = decamutexon();
        dwt_isr();
        decamutexoff(stat);
    }
}

/* GPIO ISR handler for DW1000 IRQ */
static void IRAM_ATTR dw1000_gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Notify the task */
    if (dw1000_irq_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(dw1000_irq_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_gpio_init()
 *
 * @brief Initialize GPIO pins for DW1000 (simple pin setup only)
 */
int dw1000_gpio_init(gpio_num_t io_rst, gpio_num_t io_irq, gpio_num_t io_wake)
{
    /* Save pin configuration */
    gpio_pin_rst = io_rst;
    gpio_pin_irq = io_irq;
    gpio_pin_wake = io_wake;

    /* Configure GPIO for RST - Use open-drain because RST should float high, not be driven high
     * DW1000 datasheet v2.08 §5.6.1: "The RSTn pin should not be driven high but left floating." */
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << io_rst),
        .mode = GPIO_MODE_OUTPUT_OD,       /* Open-drain output */
        .pull_up_en = GPIO_PULLUP_DISABLE, /* According to datasheet, you should NEVER PULL HIGH EXTERNALLY even though it's active low */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&rst_conf);
    gpio_set_level(io_rst, 1); /* Release RST (let it float high via DWM1000 internal pull-up) */

    /* Configure GPIO for WAKE (output) - this is optional, set to GPIO_NUM_NC (-1) if not used */
    if (io_wake != GPIO_NUM_NC)
    {
        gpio_config_t wake_conf = {
            .pin_bit_mask = (1ULL << io_wake),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
        gpio_config(&wake_conf);
        gpio_set_level(io_wake, 0); /* Initially low */
    }

    /* Configure GPIO for IRQ (input with interrupt capability) */
    gpio_config_t irq_conf = {
        .pin_bit_mask = (1ULL << io_irq),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE /* DW1000 IRQ is active high */
    };
    gpio_config(&irq_conf);

    /* Reset after pin initialization */
    dw1000_hard_reset();

    ESP_LOGI(TAG, "DW1000 GPIO initialized");
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_gpio_deinit()
 *
 * @brief De-initialize GPIO pins
 */
void dw1000_gpio_deinit(void)
{
    /* Unregister ISR if configured */
    if (dw1000_irq_task_handle != NULL)
    {
        dw1000_unregister_isr();
    }

    ESP_LOGI(TAG, "DW1000 GPIO deinitialized");
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_setup_isr()
 *
 * @brief Setup interrupt handling for DW1000
 */
int dw1000_setup_isr(uint32_t task_priority,
                     dwt_cb_t cbTxDone,
                     dwt_cb_t cbRxOk,
                     dwt_cb_t cbRxTo,
                     dwt_cb_t cbRxErr)
{
    /* Set interrupt polarity on DWM1000 to positive edge triggering */
    dwt_write32bitreg(SYS_CFG_ID, dwt_read32bitreg(SYS_CFG_ID) | SYS_CFG_HIRQ_POL);

    /* Create IRQ handling task */
    BaseType_t task_result = xTaskCreate(
        dw1000_irq_task,
        "dw1000_irq",
        4096,
        NULL,
        task_priority,
        &dw1000_irq_task_handle);

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create DW1000 IRQ task");
        return -1;
    }
    ESP_LOGI(TAG, "DW1000 IRQ task created with priority %lu", task_priority);

    /* Install GPIO ISR service if not already installed */
    esp_err_t isr_ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(isr_ret));
        vTaskDelete(dw1000_irq_task_handle);
        dw1000_irq_task_handle = NULL;
        return -1;
    }

    /* Configure interrupt type and add ISR handler */
    isr_ret = gpio_isr_handler_add(gpio_pin_irq, dw1000_gpio_isr_handler, NULL);
    if (isr_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(isr_ret));
        vTaskDelete(dw1000_irq_task_handle);
        dw1000_irq_task_handle = NULL;
        return -1;
    }

    /* Set DW1000 callbacks */
    dwt_setcallbacks(cbTxDone, cbRxOk, cbRxTo, cbRxErr);
    ESP_LOGI(TAG, "DW1000 callbacks configured");

    /* Enable interrupt */
    dw1000_irq_enabled = true;
    gpio_intr_enable(gpio_pin_irq);
    ESP_LOGI(TAG, "DW1000 ISR enabled");

    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_unregister_isr()
 *
 * @brief Unregister interrupt handling for DW1000
 */
void dw1000_unregister_isr(void)
{
    /* Disable interrupt */
    dw1000_irq_enabled = false;
    gpio_intr_disable(gpio_pin_irq);

    /* Remove ISR handler */
    gpio_isr_handler_remove(gpio_pin_irq);

    /* Delete task */
    if (dw1000_irq_task_handle != NULL)
    {
        vTaskDelete(dw1000_irq_task_handle);
        dw1000_irq_task_handle = NULL;
    }

    ESP_LOGI(TAG, "DW1000 ISR unregistered");
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_hard_reset()
 *
 * @brief Hardware reset of DW1000
 * @warning After reset and dwt_initialise(), you must call dw1000_spi_fix_bug() before normal operation
 */
void dw1000_hard_reset(void)
{
    ESP_LOGI(TAG, "Resetting DW1000...");

    /* Drive RST low */
    gpio_set_level(gpio_pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(1)); /* Hold reset low */

    /* Release RST (let it float high via pull-up) */
    gpio_set_level(gpio_pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(2)); /* Wait for DW1000 to start up */

    ESP_LOGI(TAG, "DW1000 reset complete");
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dw1000_wake_up()
 *
 * Pull wake up and hold 1ms then release to wake up DW1000 from sleep or deep sleep
 */
void dw1000_wake_up(void)
{
    if (gpio_pin_wake != GPIO_NUM_NC)
    {
        gpio_set_level(gpio_pin_wake, 1);
        vTaskDelay(pdMS_TO_TICKS(1)); /* Hold wake high for 1ms */
        gpio_set_level(gpio_pin_wake, 0);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_gpio_enable_irq()
 *
 * @brief Enable DW1000 IRQ
 */
void dw1000_gpio_enable_irq(void)
{
    dw1000_irq_enabled = true;
    gpio_intr_enable(gpio_pin_irq);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_gpio_disable_irq()
 *
 * @brief Disable DW1000 IRQ
 */
void dw1000_gpio_disable_irq(void)
{
    dw1000_irq_enabled = false;
    gpio_intr_disable(gpio_pin_irq);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_gpio_get_irq_status()
 *
 * @brief Get DW1000 IRQ status
 */
bool dw1000_gpio_get_irq_status(void)
{
    return dw1000_irq_enabled;
}