/**
 * @file DW1000Manager.hpp
 * @brief Simple UWB DS-TWR ranging manager for DW1000
 *
 * Provides a simplified interface for DS-TWR ranging with both
 * initiator and responder roles. Hides all protocol details.
 */

#ifndef DW1000_MANAGER_HPP
#define DW1000_MANAGER_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace UWBRanging
{

    /**
     * @brief Ranging measurement result
     */
    struct RangingResult
    {
        double distance_m;            // Distance in meters
        uint64_t measurement_time_us; // Measurement timestamp (microseconds since boot)
    };

    /**
     * @namespace Initiator
     * @brief Functions for the ranging initiator (triggers ranging exchanges)
     * @note Initiator only triggers ranging exchanges and does not calculate distance, cannot coexist with responder.
     */
    namespace Initiator
    {
        /**
         * @brief Initialize UWB hardware as initiator
         * @return true if successful, false otherwise
         */
        bool Initialize();

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

    /**
     * @namespace Responder
     * @brief Functions for the ranging responder (calculates and reports distance)
     * @note Responder listens for ranging requests and calculates distance, cannot coexist with initiator.
     */
    namespace Responder
    {
        /**
         * @brief Initialize UWB hardware as responder
         * @return Queue handle for ranging results (NULL if failed)
         * @note Creates internal queue with size 10, overriding oldest results
         */
        QueueHandle_t Initialize();

        /**
         * @brief Start UWB RX routine
         */
        void Begin();

        /**
         * @brief Stop UWB RX routine
         */
        void Stop();

        /**
         * @brief Check if responder is active
         * @return true if initialized and ready
         */
        bool IsActive();
    }

} // namespace UWBRanging

#endif // DW1000_MANAGER_HPP
