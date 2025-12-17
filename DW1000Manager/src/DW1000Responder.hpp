/**
 * @file DW1000Responder.hpp
 * @brief Responder interface for DW1000 DS-TWR ranging
 */

#ifndef DW1000_RESPONDER_HPP
#define DW1000_RESPONDER_HPP

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
     * @namespace Responder
     * @brief Functions for the ranging responder (calculates and reports distance)
     * @note Responder listens for ranging requests and calculates distance
     */
    namespace Responder
    {
        /**
         * @brief Initialize UWB hardware as responder
         * @param cs_pin SPI chip select pin
         * @param int_pin Interrupt pin
         * @param rst_pin Reset pin
         * @param callback_priority Priority of ISR task (default: 6)
         * @return Queue handle for ranging results (NULL if failed)
         * @note Creates internal queue with size 10, overriding oldest results
         * @note SPI bus (SPI2_HOST) must be initialized before calling this function
         */
        QueueHandle_t Initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin, uint32_t callback_priority = 6);

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

#endif // DW1000_RESPONDER_HPP
