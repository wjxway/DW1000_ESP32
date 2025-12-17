/**
 * @file DW1000Initiator.hpp
 * @brief Initiator interface for DW1000 DS-TWR ranging
 */

#ifndef DW1000_INITIATOR_HPP
#define DW1000_INITIATOR_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

namespace UWBRanging
{
    /**
     * @namespace Initiator
     * @brief Functions for the ranging initiator (triggers ranging exchanges)
     * @note Initiator only triggers ranging exchanges and does not calculate distance
     */
    namespace Initiator
    {
        /**
         * @brief Initialize UWB hardware as initiator
         * @param cs_pin SPI chip select pin
         * @param int_pin Interrupt pin
         * @param rst_pin Reset pin
         * @param callback_priority Priority of ISR task (default: 6)
         * @param ranging_priority Priority of ranging task (default: 1)
         * @return true if successful, false if failed
         * @note SPI bus (SPI2_HOST) must be initialized before calling this function
         */
        bool Initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin, uint32_t callback_priority = 6, uint32_t ranging_priority = 1);

        /**
         * @brief Start UWB TX/RX routine with specified interval
         * @param interval_ms Time interval in milliseconds for automatic ranging (0 for manual trigger only)
         */
        void Begin(uint32_t interval_ms);

        /**
         * @brief Stop UWB TX/RX routine
         */
        void Stop();

        /**
         * @brief Trigger a single ranging exchange
         * @return true if ranging was initiated successfully, false otherwise
         * @note Non-blocking, result is discarded (initiator only triggers)
         */
        bool TriggerRanging();

        /**
         * @brief Check if initiator is active
         * @return true if initialized and ready
         */
        bool IsActive();
    }

} // namespace UWBRanging

#endif // DW1000_INITIATOR_HPP
