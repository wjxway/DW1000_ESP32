/*! ----------------------------------------------------------------------------
 * @file    deca_debug.c
 * @brief   DW1000 debug register parsing functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 * Modified for ESP32-S3 using ESP-IDF
 *
 * All rights reserved.
 */

#include "deca_debug.h"
#include <stdio.h>
#include <string.h>

/* Status bit name structure for iteration */
typedef struct {
    uint32_t bit;
    const char* name;
} status_bit_info_t;

/* Status register bit names (octets 0-3) */
static const status_bit_info_t status_bits_low[] = {
    {SYS_STATUS_IRQS,       "IRQS"},
    {SYS_STATUS_CPLOCK,     "CPLOCK"},
    {SYS_STATUS_ESYNCR,     "ESYNCR"},
    {SYS_STATUS_AAT,        "AAT"},
    {SYS_STATUS_TXFRB,      "TXFRB"},
    {SYS_STATUS_TXPRS,      "TXPRS"},
    {SYS_STATUS_TXPHS,      "TXPHS"},
    {SYS_STATUS_TXFRS,      "TXFRS"},
    {SYS_STATUS_RXPRD,      "RXPRD"},
    {SYS_STATUS_RXSFDD,     "RXSFDD"},
    {SYS_STATUS_LDEDONE,    "LDEDONE"},
    {SYS_STATUS_RXPHD,      "RXPHD"},
    {SYS_STATUS_RXPHE,      "RXPHE"},
    {SYS_STATUS_RXDFR,      "RXDFR"},
    {SYS_STATUS_RXFCG,      "RXFCG"},
    {SYS_STATUS_RXFCE,      "RXFCE"},
    {SYS_STATUS_RXRFSL,     "RXRFSL"},
    {SYS_STATUS_RXRFTO,     "RXRFTO"},
    {SYS_STATUS_LDEERR,     "LDEERR"},
    {SYS_STATUS_RXOVRR,     "RXOVRR"},
    {SYS_STATUS_RXPTO,      "RXPTO"},
    {SYS_STATUS_GPIOIRQ,    "GPIOIRQ"},
    {SYS_STATUS_SLP2INIT,   "SLP2INIT"},
    {SYS_STATUS_RFPLL_LL,   "RFPLL_LL"},
    {SYS_STATUS_CLKPLL_LL,  "CLKPLL_LL"},
    {SYS_STATUS_RXSFDTO,    "RXSFDTO"},
    {SYS_STATUS_HPDWARN,    "HPDWARN"},
    {SYS_STATUS_TXBERR,     "TXBERR"},
    {SYS_STATUS_AFFREJ,     "AFFREJ"},
    {SYS_STATUS_HSRBP,      "HSRBP"},
    {SYS_STATUS_ICRBP,      "ICRBP"},
    {0, NULL}
};

/* Status register bit names (octet 4) - need to mask for comparison */
static const status_bit_info_t status_bits_high[] = {
    {0x01,  "RXRSCS"},  // Bit 0 of octet 4
    {0x02,  "RXPREJ"},  // Bit 1 of octet 4
    {0x08,  "TXPUTE"},  // Bit 3 of octet 4
    {0, NULL}
};

void deca_get_sys_status(sys_status_reg_t *status)
{
    if (status == NULL) {
        return;
    }
    
    uint8_t sys_status[5];
    dwt_readfromdevice(SYS_STATUS_ID, 0, 5, sys_status);
    
    // Parse the 5-byte register into structure
    status->status_low = ((uint32_t)sys_status[0]) |
                         ((uint32_t)sys_status[1] << 8) |
                         ((uint32_t)sys_status[2] << 16) |
                         ((uint32_t)sys_status[3] << 24);
    status->status_high = sys_status[4];
}

void deca_get_sys_state(sys_state_reg_t *state)
{
    if (state == NULL) {
        return;
    }
    
    uint8_t sys_state[5];
    dwt_readfromdevice(SYS_STATE_ID, 0, 5, sys_state);
    
    // Extract state fields from the raw bytes
    // Byte 0: bits 7:4 = TX_STATE
    // Byte 1: RX_STATE  
    // Byte 2: PMSC_STATE
    state->tx_state = (sys_state[0] >> 4) & 0x0F;
    state->rx_state = sys_state[1];
    state->pmsc_state = sys_state[2];
}

int deca_get_status_string(const sys_status_reg_t *parsed, char *buffer, int buffer_size)
{
    if (parsed == NULL || buffer == NULL || buffer_size <= 0) {
        return 0;
    }
    
    int pos = 0;
    bool first = true;
    
    buffer[0] = '\0';
    
    // Check low status bits (octets 0-3)
    for (int i = 0; status_bits_low[i].name != NULL; i++) {
        if (parsed->status_low & status_bits_low[i].bit) {
            if (!first && pos < buffer_size - 2) {
                buffer[pos++] = ',';
                buffer[pos++] = ' ';
            }
            
            int name_len = strlen(status_bits_low[i].name);
            if (pos + name_len < buffer_size) {
                strcpy(&buffer[pos], status_bits_low[i].name);
                pos += name_len;
                first = false;
            } else {
                break;  // Buffer full
            }
        }
    }
    
    // Check high status bits (octet 4)
    for (int i = 0; status_bits_high[i].name != NULL; i++) {
        if (parsed->status_high & status_bits_high[i].bit) {
            if (!first && pos < buffer_size - 2) {
                buffer[pos++] = ',';
                buffer[pos++] = ' ';
            }
            
            int name_len = strlen(status_bits_high[i].name);
            if (pos + name_len < buffer_size) {
                strcpy(&buffer[pos], status_bits_high[i].name);
                pos += name_len;
                first = false;
            } else {
                break;  // Buffer full
            }
        }
    }
    
    if (first) {
        // No flags set
        const char* none_msg = "(none)";
        int len = strlen(none_msg);
        if (len < buffer_size) {
            strcpy(buffer, none_msg);
            pos = len;
        }
    }
    
    buffer[pos] = '\0';
    return pos;
}

const char* deca_get_pmsc_state_string(uint8_t pmsc_state)
{
    switch (pmsc_state) {
        case PMSC_STATE_INIT:
            return "INIT - DW1000 is in INIT";
        case PMSC_STATE_IDLE:
            return "IDLE - DW1000 is in IDLE";
        case PMSC_STATE_TX_WAIT:
            return "TX_WAIT - DW1000 is waiting to start transmitting";
        case PMSC_STATE_RX_WAIT:
            return "RX_WAIT - DW1000 is waiting to enter receive mode";
        case PMSC_STATE_TX:
            return "TX - DW1000 is transmitting";
        case PMSC_STATE_RX:
            return "RX - DW1000 is in receive mode";
        default:
            return "UNKNOWN - Unknown PMSC state";
    }
}

const char* deca_get_rx_state_string(uint8_t rx_state)
{
    switch (rx_state) {
        case RX_STATE_IDLE:
            return "IDLE - Receiver is in idle";
        case RX_STATE_START_ANALOG:
            return "START_ANALOG - Start analog receiver blocks";
        case RX_STATE_RX_READY:
            return "RX_READY - Receiver ready";
        case RX_STATE_PREAMBLE_FIND:
            return "PREAMBLE_FIND - Receiver is waiting to detect preamble";
        case RX_STATE_PREAMBLE_TO:
            return "PREAMBLE_TO - Preamble timeout";
        case RX_STATE_SFD_FOUND:
            return "SFD_FOUND - SFD found";
        case RX_STATE_CONFIG_PHR_RX:
            return "CONFIG_PHR_RX - Configure for PHR reception";
        case RX_STATE_PHR_RX_START:
            return "PHR_RX_START - PHR reception started";
        case RX_STATE_DATA_RATE_RDY:
            return "DATA_RATE_READY - Ready for data reception";
        case RX_STATE_DATA_RX_SEQ:
            return "DATA_RX_SEQ - Data reception";
        case RX_STATE_CONFIG_DATA:
            return "CONFIG_DATA - Configure for data";
        case RX_STATE_PHR_NOT_OK:
            return "PHR_NOT_OK - PHR error";
        case RX_STATE_LAST_SYMBOL:
            return "LAST_SYMBOL - Received last symbol";
        case RX_STATE_WAIT_RSD_DONE:
            return "WAIT_RSD_DONE - Wait for Reed Solomon decoder to finish";
        case RX_STATE_RSD_OK:
            return "RSD_OK - Reed Solomon correct";
        case RX_STATE_RSD_NOT_OK:
            return "RSD_NOT_OK - Reed Solomon error";
        case RX_STATE_RECONFIG_110:
            return "RECONFIG_110 - Reconfigure for 110 kbps data";
        case RX_STATE_WAIT_110_PHR:
            return "WAIT_110_PHR - Wait for 110 kbps PHR";
        default:
            return "UNKNOWN - Unknown RX state";
    }
}

const char* deca_get_tx_state_string(uint8_t tx_state)
{
    switch (tx_state) {
        case TX_STATE_IDLE:
            return "IDLE - Transmitter is in idle";
        case TX_STATE_PREAMBLE:
            return "PREAMBLE - Transmitting preamble";
        case TX_STATE_SFD:
            return "SFD - Transmitting SFD";
        case TX_STATE_PHR:
            return "PHR - Transmitting PHR";
        case TX_STATE_SDE:
            return "SDE - Transmitting PHR parity SECDED bits";
        case TX_STATE_DATA:
            return "DATA - Transmitting data";
        case TX_STATE_RSP_DATA:
            return "RSP_DATA - Transmitting Reed Solomon parity block";
        default:
            return "UNKNOWN - Unknown TX state";
    }
}

int deca_get_state_string(const sys_state_reg_t *parsed, char *buffer, int buffer_size)
{
    if (parsed == NULL || buffer == NULL || buffer_size <= 0) {
        return 0;
    }
    
    int written = snprintf(buffer, buffer_size,
                          "PMSC: %s | RX: %s | TX: %s",
                          deca_get_pmsc_state_string(parsed->pmsc_state),
                          deca_get_rx_state_string(parsed->rx_state),
                          deca_get_tx_state_string(parsed->tx_state));
    
    if (written < 0) {
        buffer[0] = '\0';
        return 0;
    }
    
    if (written >= buffer_size) {
        return buffer_size - 1;  // Truncated
    }
    
    return written;
}
