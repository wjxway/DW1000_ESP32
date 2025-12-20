/**
 * @file DW1000Responder.cpp
 * @brief Responder implementation for DW1000 DS-TWR ranging
 */

#include "DW1000Responder.hpp"
#include "DW1000Common.hpp"

extern "C"
{
#include "platform/deca_spi.h"
#include "platform/deca_gpio.h"
#include "decadriver/deca_device_api.h"
}

namespace UWBRanging
{
    namespace Responder
    {
        using namespace UWBRanging;

        /* State variables */
        static bool initialized = false;
        static bool running = false;
        static uint8 frame_seq_nb = 0;
        static QueueHandle_t result_queue = nullptr;

        /* Timestamps */
        static uint64_t poll_rx_ts;
        static uint64_t resp_tx_ts;
        static uint64_t final_rx_ts;

        /* Delays and timeouts */
        constexpr uint16_t POLL_RX_TO_RESP_TX_DLY_UUS = 3000;
        constexpr uint16_t RESP_TX_TO_FINAL_RX_DLY_UUS = 2000;
        constexpr uint16_t FINAL_RX_TIMEOUT_UUS = 5000;

        /* Queue configuration */
        constexpr uint8_t QUEUE_SIZE = 10;

        /* Latest data and optimistic lock */
        RangingResult latest_result;
        uint32_t result_update_count = 0;

        /* State machine */
        enum State
        {
            WAITING_POLL,
            POLL_RECEIVED,
            RESP_SENT
        };

        static volatile State current_state = WAITING_POLL;

        static void ResetState()
        {
            current_state = WAITING_POLL;
            dwt_setrxtimeout(0);
            dwt_forcetrxoff();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        static void CalculateAndReportDistance()
        {
            uint32 poll_tx_ts_32, resp_rx_ts_32, final_tx_ts_32;
            uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;

            resp_tx_ts = GetTxTimestamp();
            final_rx_ts = GetRxTimestamp();

            GetTimestampFromMessage(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts_32);
            GetTimestampFromMessage(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts_32);
            GetTimestampFromMessage(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_32);

            poll_rx_ts_32 = (uint32)poll_rx_ts;
            resp_tx_ts_32 = (uint32)resp_tx_ts;
            final_rx_ts_32 = (uint32)final_rx_ts;

            double Ra = (double)(resp_rx_ts_32 - poll_tx_ts_32);
            double Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
            double Da = (double)(final_tx_ts_32 - resp_rx_ts_32);
            double Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);

            int64_t tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
            double tof = tof_dtu * DWT_TIME_UNITS;
            double distance = tof * SPEED_OF_LIGHT;

            /* Update latest result with optimistic lock */
            latest_result.distance_m = distance;
            latest_result.measurement_time_us = esp_timer_get_time();
            result_update_count++;

            /* Send result to queue */
            if (result_queue != nullptr)
            {
                if (xQueueSend(result_queue, &latest_result, 0) != pdTRUE)
                {
                    /* Queue full, remove oldest and retry */
                    RangingResult dummy;
                    xQueueReceive(result_queue, &dummy, 0);
                    xQueueSend(result_queue, &latest_result, 0);
                }
            }

#if DEBUG_CALLBACKS
            Serial.printf("[DIST] Distance calculated: %.3f m (seq=%d)\n", distance, frame_seq_nb - 1);
#endif
        }

        /* Callback handlers */
        static void RxOkCallback(const dwt_cb_data_t *cb_data)
        {
            if (cb_data->datalength > RX_BUF_LEN)
            {
#if DEBUG_CALLBACKS
                Serial.printf("[RXOK] Buffer overflow @ %lld\n", esp_timer_get_time());
                PrintStatusState("RXOK");
#endif
                ResetState();
                return;
            }

            dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (current_state == WAITING_POLL && memcmp(rx_buffer, poll_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                current_state = POLL_RECEIVED;
                poll_rx_ts = GetRxTimestamp();

                uint32 resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(RESP_MSG_LEN, resp_msg, 0);
                dwt_writetxfctrl(RESP_MSG_LEN, 0, 1);

                // Set expected delay and timeout for final message reception
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS)
                {
                    frame_seq_nb++;
#if DEBUG_CALLBACKS
                    Serial.printf("[RXOK] Poll received, response sent (seq=%d) @ %lld\n", frame_seq_nb - 1, esp_timer_get_time());
                    PrintStatusState("RXOK");
#endif
                }
                else
                {
#if DEBUG_CALLBACKS
                    Serial.printf("[RXOK] Response TX failed @ %lld\n", esp_timer_get_time());
                    PrintStatusState("RXOK");
#endif
                    ResetState();
                }
            }
            else if (current_state == RESP_SENT && memcmp(rx_buffer, final_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                CalculateAndReportDistance();
#if DEBUG_CALLBACKS
                Serial.printf("[RXOK] Final received (seq=%d) @ %lld\n", frame_seq_nb - 1, esp_timer_get_time());
                PrintStatusState("RXOK");
#endif
                ResetState();
            }
            else
            {
#if DEBUG_CALLBACKS
                Serial.printf("[RXOK] Unexpected message/state=%d @ %lld\n", current_state, esp_timer_get_time());
                PrintStatusState("RXOK");
#endif
                ResetState();
            }
        }

        static void RxTimeoutCallback(const dwt_cb_data_t *cb_data)
        {
#if DEBUG_CALLBACKS
            Serial.printf("[RXTO] RX timeout : 0x%08X @ %lld\n", cb_data->status, esp_timer_get_time());
            PrintStatusState("RXTO");
#endif
            ResetState();
        }

        static void RxErrorCallback(const dwt_cb_data_t *cb_data)
        {
#if DEBUG_CALLBACKS
            Serial.printf("[RXERR] RX error : 0x%08X @ %lld\n", cb_data->status, esp_timer_get_time());
            PrintStatusState("RXERR");
#endif
            ResetState();
        }

        static void TxConfirmCallback(const dwt_cb_data_t *cb_data)
        {
#if DEBUG_CALLBACKS
            Serial.printf("[TXCONF] TX confirmed : 0x%08X @ %lld\n", cb_data->status, esp_timer_get_time());
            PrintStatusState("TXCONF");
#endif

            current_state = RESP_SENT;

            int r = dwt_rxenable(DWT_START_RX_DELAYED);
#if DEBUG_CALLBACKS
            if (r != DWT_SUCCESS)
            {
                Serial.printf("[TXCONF] RX enable failed: 0x%08X @ %lld\n", cb_data->status, esp_timer_get_time());
            }
#endif
        }

        QueueHandle_t Initialize(uint8_t cs_pin, uint8_t int_pin, uint8_t rst_pin, uint32_t callback_priority)
        {
            decaIrqStatus_t stat = decamutexon();

            // when already initialized, only execute reset routine
            if (!initialized)
            {
                result_queue = xQueueCreate(QUEUE_SIZE, sizeof(RangingResult));
                if (result_queue == nullptr)
                {
                    decamutexoff(stat);
                    return nullptr;
                }

#if DEBUG_INITIALIZATION
                Serial.println("[INIT] Initializing UWB Responder");
#endif

                /* Initialize DW1000 SPI device */
                if (dw1000_spi_init(UWB_SPI_HOST, (gpio_num_t)cs_pin, nullptr) != 0)
                {
#if DEBUG_INITIALIZATION
                    Serial.println("[INIT] ERROR: SPI init failed");
#endif
                    vQueueDelete(result_queue);
                    result_queue = nullptr;
                    decamutexoff(stat);
                    return nullptr;
                }

                if (dw1000_gpio_init((gpio_num_t)rst_pin, (gpio_num_t)int_pin, GPIO_NUM_NC) != 0)
                {
#if DEBUG_INITIALIZATION
                    Serial.println("[INIT] ERROR: GPIO init failed");
#endif
                    vQueueDelete(result_queue);
                    result_queue = nullptr;
                    decamutexoff(stat);
                    return nullptr;
                }
            }
            else
            {
                // disable isr first
                dw1000_unregister_isr();
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
                vQueueDelete(result_queue);
                result_queue = nullptr;
                decamutexoff(stat);
                return nullptr;
            }

            if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
            {
#if DEBUG_INITIALIZATION
                Serial.println("[INIT] ERROR: dwt_initialise failed");
#endif
                vQueueDelete(result_queue);
                result_queue = nullptr;
                decamutexoff(stat);
                return nullptr;
            }

            spi_set_rate_high();

            delayMicroseconds(100);

            dwt_configure(&dw1000_config);
            dwt_setrxantennadelay(RX_ANT_DLY);
            dwt_settxantennadelay(TX_ANT_DLY);

            if (dw1000_setup_isr(callback_priority, &TxConfirmCallback, &RxOkCallback, &RxTimeoutCallback, &RxErrorCallback) != 0)
            {
#if DEBUG_INITIALIZATION
                Serial.println("[INIT] ERROR: ISR setup failed");
#endif
                vQueueDelete(result_queue);
                result_queue = nullptr;
                decamutexoff(stat);
                return nullptr;
            }

            Serial.println("[INIT] DW1000 ISR initialized");

            Serial.printf("[INIT] devid 0x%08X\n", dwt_readdevid());
            Serial.printf("[INIT] devid 0x%08X\n", dwt_readdevid());

            dwt_write32bitreg(SYS_MASK_ID, DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE |
                                               DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXOVRR | DWT_INT_RXPTO | DWT_INT_SFDT); // New value

            Serial.println("[INIT] DW1000 write done");

            dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE |
                                 DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXOVRR | DWT_INT_RXPTO | DWT_INT_SFDT,
                             2);

            initialized = true;

#if DEBUG_INITIALIZATION
            Serial.println("[INIT] UWB Responder initialized successfully");
#endif

            dwt_forcetrxoff();

            decamutexoff(stat);

            return result_queue;
        }

        void Begin()
        {
            if (!initialized || running)
                return;

            decaIrqStatus_t stat = decamutexon();
            ResetState();
            decamutexoff(stat);
            running = true;

#if DEBUG_INITIALIZATION
            Serial.println("[BEGIN] Started RX routine");
#endif
        }

        void Stop()
        {
            if (!initialized || !running)
                return;

            running = false;

            decaIrqStatus_t stat = decamutexon();
            dwt_forcetrxoff();
            decamutexoff(stat);

#if DEBUG_INITIALIZATION
            Serial.println("[STOP] Stopped RX routine");
#endif
        }

        bool IsActive()
        {
            return initialized && running;
        }

        RangingResult GetLatestResult()
        {
            RangingResult result;
            uint32_t before_count;

            do
            {
                before_count = result_update_count;
                result = latest_result;
            } while (before_count != result_update_count);

            return result;
        }

    } // namespace Responder

} // namespace UWBRanging