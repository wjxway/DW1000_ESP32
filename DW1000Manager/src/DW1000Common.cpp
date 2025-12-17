/**
 * @file DW1000Common.cpp
 * @brief Common implementations for DW1000 DS-TWR ranging
 */

#include "DW1000Common.hpp"

extern "C"
{
#include "platform/deca_spi.h"
#include "platform/deca_debug.h"
}

namespace UWBRanging
{
    /* DW1000 configuration */
    dwt_config_t dw1000_config = {
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

    /* Frame definitions */
    uint8 poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
    uint8 resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
    uint8 final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    /* RX buffer */
    uint8 rx_buffer[RX_BUF_LEN];

    uint64_t GetTxTimestamp()
    {
        uint8 ts_tab[5];
        uint64_t ts = 0;
        dwt_readtxtimestamp(ts_tab);
        for (int i = 4; i >= 0; i--)
        {
            ts <<= 8;
            ts |= ts_tab[i];
        }
        return ts;
    }

    uint64_t GetRxTimestamp()
    {
        uint8 ts_tab[5];
        uint64_t ts = 0;
        dwt_readrxtimestamp(ts_tab);
        for (int i = 4; i >= 0; i--)
        {
            ts <<= 8;
            ts |= ts_tab[i];
        }
        return ts;
    }

    void SetTimestampInMessage(uint8 *ts_field, uint64_t ts)
    {
        for (int i = 0; i < FINAL_MSG_TS_LEN; i++)
        {
            ts_field[i] = (uint8)ts;
            ts >>= 8;
        }
    }

    void GetTimestampFromMessage(const uint8 *ts_field, uint32 *ts)
    {
        *ts = 0;
        for (int i = 0; i < FINAL_MSG_TS_LEN; i++)
        {
            *ts += ts_field[i] << (i * 8);
        }
    }

    void PrintStatusState(const char *prefix)
    {
        sys_status_reg_t status;
        sys_state_reg_t state;
        char status_buf[512], state_buf[512];

        deca_get_sys_status(&status);
        deca_get_sys_state(&state);
        deca_get_status_string(&status, status_buf, sizeof(status_buf));
        deca_get_state_string(&state, state_buf, sizeof(state_buf));

        Serial.printf("[%s] Status: 0x%02X %08X %s\n", prefix, status.status_high, status.status_low, status_buf);
        Serial.printf("[%s] State: %s\n", prefix, state_buf);
    }

} // namespace UWBRanging
