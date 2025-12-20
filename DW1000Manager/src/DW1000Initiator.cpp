/**
 * @file DW1000Initiator.cpp
 * @brief Initiator implementation for DW1000 DS-TWR ranging
 */

#include "DW1000Initiator.hpp"
#include "DW1000Common.hpp"

extern "C"
{
#include "platform/deca_spi.h"
#include "platform/deca_gpio.h"
#include "decadriver/deca_device_api.h"
}

namespace UWBRanging
{
    namespace Initiator
    {
        using namespace UWBRanging;

        /* State variables */
        static bool initialized = false;
        static bool running = false;
        static uint8 frame_seq_nb = 0;
        static volatile bool ranging_in_progress = false;
        static uint32_t ranging_interval_ms = 0;
        static TaskHandle_t ranging_task_handle = nullptr;
        static uint32_t ranging_task_priority = 1;

        /* Timestamps */
        static uint64_t poll_tx_ts;
        static uint64_t resp_rx_ts;
        static uint64_t final_tx_ts;

        /* Delays and timeouts */
        constexpr uint16_t POLL_TX_TO_RESP_RX_DLY_UUS = 2000;
        constexpr uint16_t RESP_RX_TO_FINAL_TX_DLY_UUS = 3000;
        constexpr uint16_t RESP_RX_TIMEOUT_UUS = 5000;

        /* State machine */
        enum State
        {
            IDLE,
            POLL_SENT,
            RESP_RECEIVED
        };

        static volatile State current_state = IDLE;

        static void ResetState()
        {
            decaIrqStatus_t stat = decamutexon();
            current_state = IDLE;
            ranging_in_progress = false;
            dwt_setrxtimeout(0);
            dwt_forcetrxoff();
            decamutexoff(stat);
        }

        /* Callback handlers */
        static void RxOkCallback(const dwt_cb_data_t *cb_data)
        {
            if (current_state != POLL_SENT)
            {
                ResetState();
#if DEBUG_CALLBACKS
                Serial.printf("[RXOK] Unexpected state @ %lld\n", esp_timer_get_time());
                PrintStatusState("RXOK");
#endif
                return;
            }

            if (cb_data->datalength > RX_BUF_LEN)
            {
                ResetState();
#if DEBUG_CALLBACKS
                Serial.printf("[RXOK] Buffer overflow @ %lld\n", esp_timer_get_time());
                PrintStatusState("RXOK");
#endif
                return;
            }

            dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, resp_msg, ALL_MSG_COMMON_LEN) != 0)
            {
                ResetState();
#if DEBUG_CALLBACKS
                Serial.printf("[RXOK] Invalid response @ %lld\n", esp_timer_get_time());
                PrintStatusState("RXOK");
#endif
                return;
            }

            current_state = RESP_RECEIVED;

            poll_tx_ts = GetTxTimestamp();
            resp_rx_ts = GetRxTimestamp();

            uint32 final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(final_tx_time);

            final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            SetTimestampInMessage(&final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
            SetTimestampInMessage(&final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
            SetTimestampInMessage(&final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

            final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(FINAL_MSG_LEN, final_msg, 0);
            dwt_writetxfctrl(FINAL_MSG_LEN, 0, 1);

            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS)
            {
                frame_seq_nb++;
#if DEBUG_CALLBACKS
                Serial.printf("[RXOK] Response received, final sent (seq=%d) @ %lld\n", frame_seq_nb - 1, esp_timer_get_time());
                PrintStatusState("RXOK");
#endif
            }
            else
            {
                ResetState();
#if DEBUG_CALLBACKS
                Serial.printf("[RXOK] Final TX failed @ %lld\n", esp_timer_get_time());
                PrintStatusState("RXOK");
#endif
            }
        }

        static void RxTimeoutCallback(const dwt_cb_data_t *cb_data)
        {
            ResetState();
#if DEBUG_CALLBACKS
            Serial.printf("[RXTO] RX timeout @ %lld\n", esp_timer_get_time());
            PrintStatusState("RXTO");
#endif
        }

        static void RxErrorCallback(const dwt_cb_data_t *cb_data)
        {
            ResetState();
#if DEBUG_CALLBACKS
            Serial.printf("[RXERR] RX error @ %lld\n", esp_timer_get_time());
            PrintStatusState("RXERR");
#endif
        }

        static void TxConfirmCallback(const dwt_cb_data_t *cb_data)
        {
            if (current_state == IDLE)
            {
                current_state = POLL_SENT;
            }
            else
            {
                current_state = IDLE;
            }

            // Only enable RX if we just sent a POLL frame
            if (current_state == POLL_SENT)
            {
                if (dwt_rxenable(DWT_START_RX_DELAYED) != DWT_SUCCESS)
                {
#if DEBUG_CALLBACKS
                    Serial.printf("[TXCONF] ERROR: dwt_rxenable failed @ %lld\n", esp_timer_get_time());
#endif
                }
                else
                {
#if DEBUG_CALLBACKS
                    Serial.printf("[TXCONF] RX enabled after POLL sent @ %lld\n", esp_timer_get_time());
#endif
                }
            }
            else
            {
                ranging_in_progress = false;
#if DEBUG_CALLBACKS
                Serial.printf("[TXCONF] Final sent, ranging complete @ %lld\n", esp_timer_get_time());
#endif
            }

#if DEBUG_CALLBACKS
            PrintStatusState("TXCONF");
#endif
        }

        /* Ranging task for automatic polling */
        static void RangingTask(void *pvParameters)
        {
            constexpr uint32_t num_failure_to_reset = 2;
            uint32_t tx_failure_count = 0;

            while (running)
            {
                if (TriggerRanging())
                {
                    tx_failure_count = 0;
                }
                else if (tx_failure_count + 1 >= num_failure_to_reset)
                {
                    tx_failure_count = 0;
#if DEBUG_CALLBACKS
                    Serial.printf("[TRIG] TriggerRanging failed for %d times, resetting\n", num_failure_to_reset);
                    PrintStatusState("TRIG");
#endif
                    ResetState();
                }
                else
                {
                    tx_failure_count++;
                }

                vTaskDelay(pdMS_TO_TICKS(ranging_interval_ms));
            }
            ranging_task_handle = nullptr;
            vTaskDelete(nullptr);
        }

        bool Initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin, uint32_t callback_priority, uint32_t ranging_priority)
        {
            decaIrqStatus_t stat = decamutexon();

            if (!initialized)
            {
                ranging_task_priority = ranging_priority;

#if DEBUG_INITIALIZATION
                Serial.println("[INIT] Initializing UWB Initiator");
#endif

                /* Initialize DW1000 SPI device */
                if (dw1000_spi_init(UWB_SPI_HOST, (gpio_num_t)cs_pin, nullptr) != 0)
                {
#if DEBUG_INITIALIZATION
                    Serial.println("[INIT] ERROR: SPI init failed");
#endif
                    decamutexoff(stat);
                    return false;
                }

                if (dw1000_gpio_init((gpio_num_t)rst_pin, (gpio_num_t)int_pin, GPIO_NUM_NC) != 0)
                {
#if DEBUG_INITIALIZATION
                    Serial.println("[INIT] ERROR: GPIO init failed");
#endif
                    decamutexoff(stat);
                    return false;
                }
            }

            dw1000_hard_reset();
            constexpr int MAX_TRIES = 10;
            constexpr uint32_t correct_devid = 0xDECA0130;
            int tries = 0;
            while (++tries <= MAX_TRIES)
            {
                if (dwt_readdevid() == correct_devid)
                {
                    break;
                }
                dw1000_spi_fix_bug();
                delayMicroseconds(10);
            }

            if (dwt_readdevid() != correct_devid)
            {
#if DEBUG_INITIALIZATION
                Serial.println("[INIT] ERROR: Device ID read failed");
#endif
                decamutexoff(stat);
                return false;
            }

            if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
            {
#if DEBUG_INITIALIZATION
                Serial.println("[INIT] ERROR: dwt_initialise failed");
#endif
                decamutexoff(stat);
                return false;
            }

            spi_set_rate_high();

            dwt_configure(&dw1000_config);
            dwt_setrxantennadelay(RX_ANT_DLY);
            dwt_settxantennadelay(TX_ANT_DLY);

            if (dw1000_setup_isr(callback_priority, &TxConfirmCallback, &RxOkCallback, &RxTimeoutCallback, &RxErrorCallback) != 0)
            {
#if DEBUG_INITIALIZATION
                Serial.println("[INIT] ERROR: ISR setup failed");
#endif
                decamutexoff(stat);
                return false;
            }

            dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE |
                                 DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXOVRR | DWT_INT_RXPTO | DWT_INT_SFDT,
                             1);

            initialized = true;

#if DEBUG_INITIALIZATION
            Serial.println("[INIT] UWB Initiator initialized successfully");
#endif

            dwt_forcetrxoff();

            decamutexoff(stat);

            return true;
        }

        bool TriggerRanging()
        {
#if DEBUG_CALLBACKS
            Serial.printf("[TRIG] Ranging trigger started (seq=%d)\n", frame_seq_nb);
#endif

            if (!initialized || ranging_in_progress)
                return false;

            ranging_in_progress = true;
            current_state = IDLE;

            decaIrqStatus_t stat = decamutexon();

            // Force TRX off before starting new transmission
            dwt_forcetrxoff();

            poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(POLL_MSG_LEN, poll_msg, 0);
            dwt_writetxfctrl(POLL_MSG_LEN, 0, 1);

            // Set expected delay and timeout for response message reception
            dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
            dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

            // Start transmission with response expected
            int tx_ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

            decamutexoff(stat);

            if (tx_ret == DWT_SUCCESS)
            {
                frame_seq_nb++;
#if DEBUG_CALLBACKS
                Serial.printf("[TRIG] Initiated ranging #%lu, sent POLL\n", frame_seq_nb + 1);
#endif
            }
            else
            {
#if DEBUG_CALLBACKS
                Serial.printf("[TRIG] ERROR: dwt_starttx failed: %d\n", tx_ret);
#endif
            }

            return true;
        }

        void Begin(uint32_t interval_ms)
        {
            if (!initialized || running)
                return;

            ranging_interval_ms = interval_ms;
            running = true;

            xTaskCreate(RangingTask, "RangingTask", 4096, nullptr, ranging_task_priority, &ranging_task_handle);

#if DEBUG_INITIALIZATION
            Serial.printf("[BEGIN] Started with interval %u ms\n", interval_ms);
#endif
        }

        void Stop()
        {
            if (!initialized || !running)
                return;

            running = false;
            ranging_interval_ms = 0;

            while (ranging_task_handle != nullptr)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            decaIrqStatus_t stat = decamutexon();
            dwt_forcetrxoff();
            decamutexoff(stat);

#if DEBUG_INITIALIZATION
            Serial.println("[STOP] Stopped");
#endif
        }

        bool IsActive()
        {
            return initialized && running;
        }

    } // namespace Initiator

} // namespace UWBRanging