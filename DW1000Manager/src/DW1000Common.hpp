/**
 * @file DW1000Common.hpp
 * @brief Common constants and definitions for DW1000 DS-TWR ranging
 */

#ifndef DW1000_COMMON_HPP
#define DW1000_COMMON_HPP

#include <Arduino.h>
#include <driver/spi_master.h>

extern "C"
{
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
}

/* Debug macros - disabled by default, set to 1 to enable */
#define DEBUG_INITIALIZATION 1
#define DEBUG_CALLBACKS 0
#define DEBUG_STATUS_STATE 0

namespace UWBRanging
{
    /* SPI Host */
    constexpr spi_host_device_t UWB_SPI_HOST = SPI2_HOST;

    /* Fixed configuration matching DS-TWR test examples */
    extern dwt_config_t dw1000_config;

    /* Default antenna delay values for 64 MHz PRF */
    constexpr uint16_t TX_ANT_DLY = 16436;
    constexpr uint16_t RX_ANT_DLY = 16436;

    /* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor */
    constexpr uint32_t UUS_TO_DWT_TIME = 65536;

    /* Speed of light in air, in metres per second */
    constexpr uint32_t SPEED_OF_LIGHT = 299702547;

    /* Frame definitions */
    constexpr uint8_t POLL_MSG_LEN = 12;
    constexpr uint8_t RESP_MSG_LEN = 15;
    constexpr uint8_t FINAL_MSG_LEN = 24;

    extern uint8 poll_msg[POLL_MSG_LEN];
    extern uint8 resp_msg[RESP_MSG_LEN];
    extern uint8 final_msg[FINAL_MSG_LEN];

    /* Message field indexes */
    constexpr uint8_t ALL_MSG_COMMON_LEN = 10;
    constexpr uint8_t ALL_MSG_SN_IDX = 2;
    constexpr uint8_t FINAL_MSG_POLL_TX_TS_IDX = 10;
    constexpr uint8_t FINAL_MSG_RESP_RX_TS_IDX = 14;
    constexpr uint8_t FINAL_MSG_FINAL_TX_TS_IDX = 18;
    constexpr uint8_t FINAL_MSG_TS_LEN = 4;

    /* RX buffer */
    constexpr uint8_t RX_BUF_LEN = 24;
    extern uint8 rx_buffer[RX_BUF_LEN];

    /**
     * @brief Get TX timestamp from DW1000
     * @return 40-bit timestamp value
     */
    uint64_t GetTxTimestamp();

    /**
     * @brief Get RX timestamp from DW1000
     * @return 40-bit timestamp value
     */
    uint64_t GetRxTimestamp();

    /**
     * @brief Set timestamp in message buffer
     * @param ts_field Pointer to timestamp field in message
     * @param ts Timestamp value to set
     */
    void SetTimestampInMessage(uint8 *ts_field, uint64_t ts);

    /**
     * @brief Get timestamp from message buffer
     * @param ts_field Pointer to timestamp field in message
     * @param ts Pointer to store extracted timestamp
     */
    void GetTimestampFromMessage(const uint8 *ts_field, uint32 *ts);

    /**
     * @brief Print DW1000 status and state for debugging
     * @param prefix Prefix string for debug output
     */
    void PrintStatusState(const char *prefix);

} // namespace UWBRanging

#endif // DW1000_COMMON_HPP
