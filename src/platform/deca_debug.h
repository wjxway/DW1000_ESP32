/*! ----------------------------------------------------------------------------
 * @file    deca_debug.h
 * @brief   DW1000 debug register parsing functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 * Modified for ESP32-S3 using ESP-IDF
 *
 * All rights reserved.
 */

#ifndef _DECA_DEBUG_H_
#define _DECA_DEBUG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "../decadriver/deca_regs.h"
#include "../decadriver/deca_device_api.h"

/* SYS_STATE Register Bit Masks and Shifts */
#define SYS_STATE_TX_STATE_MASK 0x000000F0UL
#define SYS_STATE_TX_STATE_SHIFT 4
#define SYS_STATE_RX_STATE_MASK 0x0000FF00UL
#define SYS_STATE_RX_STATE_SHIFT 8
#define SYS_STATE_PMSC_STATE_MASK 0x00FF0000UL
#define SYS_STATE_PMSC_STATE_SHIFT 16

    /* PMSC_STATE Values */
    typedef enum
    {
        PMSC_STATE_INIT = 0x0,    // DW1000 is in INIT
        PMSC_STATE_IDLE = 0x1,    // DW1000 is in IDLE
        PMSC_STATE_TX_WAIT = 0x2, // DW1000 is waiting to start transmitting
        PMSC_STATE_RX_WAIT = 0x3, // DW1000 is waiting to enter receive mode
        PMSC_STATE_TX = 0x4,      // DW1000 is transmitting
        PMSC_STATE_RX = 0x5       // DW1000 is in receive mode
    } pmsc_state_e;

    /* RX_STATE Values */
    typedef enum
    {
        RX_STATE_IDLE = 0x00,          // Receiver is in idle
        RX_STATE_START_ANALOG = 0x01,  // Start analog receiver blocks
        RX_STATE_RX_READY = 0x04,      // Receiver ready
        RX_STATE_PREAMBLE_FIND = 0x05, // Receiver is waiting to detect preamble
        RX_STATE_PREAMBLE_TO = 0x06,   // Preamble timeout
        RX_STATE_SFD_FOUND = 0x07,     // SFD found
        RX_STATE_CONFIG_PHR_RX = 0x08, // Configure for PHR reception
        RX_STATE_PHR_RX_START = 0x09,  // PHR reception started
        RX_STATE_DATA_RATE_RDY = 0x0A, // Ready for data reception
        RX_STATE_DATA_RX_SEQ = 0x0C,   // Data reception
        RX_STATE_CONFIG_DATA = 0x0D,   // Configure for data
        RX_STATE_PHR_NOT_OK = 0x0E,    // PHR error
        RX_STATE_LAST_SYMBOL = 0x0F,   // Received last symbol
        RX_STATE_WAIT_RSD_DONE = 0x10, // Wait for Reed Solomon decoder to finish
        RX_STATE_RSD_OK = 0x11,        // Reed Solomon correct
        RX_STATE_RSD_NOT_OK = 0x12,    // Reed Solomon error
        RX_STATE_RECONFIG_110 = 0x13,  // Reconfigure for 110 kbps data
        RX_STATE_WAIT_110_PHR = 0x14   // Wait for 110 kbps PHR
    } rx_state_e;

    /* TX_STATE Values */
    typedef enum
    {
        TX_STATE_IDLE = 0x0,     // Transmitter is in idle
        TX_STATE_PREAMBLE = 0x1, // Transmitting preamble
        TX_STATE_SFD = 0x2,      // Transmitting SFD
        TX_STATE_PHR = 0x3,      // Transmitting PHR
        TX_STATE_SDE = 0x4,      // Transmitting PHR parity SECDED bits
        TX_STATE_DATA = 0x5,     // Transmitting data
        TX_STATE_RSP_DATA = 0x6  // Transmitting Reed Solomon parity block
    } tx_state_e;

    /* Status Register Structure */
    typedef struct
    {
        uint32_t status_low; // REG:0F:00 - octets 0-3
        uint8_t status_high; // REG:0F:04 - octet 4
    } sys_status_reg_t;

    /* State Register Structure */
    typedef struct
    {
        uint8_t tx_state;   // Transmitter state
        uint8_t rx_state;   // Receiver state
        uint8_t pmsc_state; // Power management state
    } sys_state_reg_t;

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn deca_get_sys_status()
     *
     * @brief Read and parse the SYS_STATUS register directly from the DW1000
     *
     * @param status - pointer to structure to fill with parsed status values
     *
     * @return none
     */
    void deca_get_sys_status(sys_status_reg_t *status);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn deca_get_sys_state()
     *
     * @brief Read and parse the SYS_STATE register directly from the DW1000
     *
     * @param state - pointer to structure to fill with parsed state values
     *
     * @return none
     */
    void deca_get_sys_state(sys_state_reg_t *state);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn deca_get_status_string()
     *
     * @brief Get a human-readable string describing active status flags
     *
     * @param parsed - pointer to parsed status register structure
     * @param buffer - buffer to write the string to
     * @param buffer_size - size of the buffer
     *
     * @return number of characters written (excluding null terminator)
     */
    int deca_get_status_string(const sys_status_reg_t *parsed, char *buffer, int buffer_size);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn deca_get_pmsc_state_string()
     *
     * @brief Get a human-readable string for overall chip state
     *
     * @param pmsc_state - overall chip state value
     *
     * @return pointer to constant string describing the state
     */
    const char *deca_get_pmsc_state_string(uint8_t pmsc_state);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn deca_get_rx_state_string()
     *
     * @brief Get a human-readable string for RX state
     *
     * @param rx_state - RX state value
     *
     * @return pointer to constant string describing the state
     */
    const char *deca_get_rx_state_string(uint8_t rx_state);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn deca_get_tx_state_string()
     *
     * @brief Get a human-readable string for TX state
     *
     * @param tx_state - TX state value
     *
     * @return pointer to constant string describing the state
     */
    const char *deca_get_tx_state_string(uint8_t tx_state);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn deca_get_state_string()
     *
     * @brief Get a complete human-readable string for all state fields
     *
     * @param parsed - pointer to parsed state register structure
     * @param buffer - buffer to write the string to
     * @param buffer_size - size of the buffer
     *
     * @return number of characters written (excluding null terminator)
     */
    int deca_get_state_string(const sys_state_reg_t *parsed, char *buffer, int buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_DEBUG_H_ */
