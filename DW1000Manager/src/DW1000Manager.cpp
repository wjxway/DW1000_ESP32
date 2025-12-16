/**
 * @file DW1000Manager.cpp
 * @brief Implementation of UWB DS-TWR ranging manager
 */

#include "DW1000Manager.hpp"
#include <HardwareDefs.hpp>

extern "C"
{
#include "platform/deca_spi.h"
#include "platform/deca_gpio.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
}

/* Debug macros - disabled by default */
#define DEBUG_INIT 0
#define DEBUG_RANGING 0

namespace UWBRanging
{
    /* SPI Host */
    constexpr spi_host_device_t SPI_HOST = SPI2_HOST;

    /* Fixed configuration matching DS-TWR test examples */
    static dwt_config_t dw1000_config = {
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

    /* Default antenna delay values for 64 MHz PRF */
    constexpr uint16_t TX_ANT_DLY = 16436;
    constexpr uint16_t RX_ANT_DLY = 16436;

    /* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor */
    constexpr uint32_t UUS_TO_DWT_TIME = 65536;

    /* Speed of light in air, in metres per second */
    constexpr uint32_t SPEED_OF_LIGHT = 299702547;

    /* Frame definitions */
    static uint8 poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
    static uint8 resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
    static uint8 final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    /* Message field indexes */
    constexpr uint8_t ALL_MSG_COMMON_LEN = 10;
    constexpr uint8_t ALL_MSG_SN_IDX = 2;
    constexpr uint8_t FINAL_MSG_POLL_TX_TS_IDX = 10;
    constexpr uint8_t FINAL_MSG_RESP_RX_TS_IDX = 14;
    constexpr uint8_t FINAL_MSG_FINAL_TX_TS_IDX = 18;
    constexpr uint8_t FINAL_MSG_TS_LEN = 4;

    /* RX buffer */
    constexpr uint8_t RX_BUF_LEN = 24;
    static uint8 rx_buffer[RX_BUF_LEN];

    /* Shared helper functions */
    static uint64_t GetTxTimestamp(void)
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

    static uint64_t GetRxTimestamp(void)
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

    static void SetTimestampInMessage(uint8 *ts_field, uint64_t ts)
    {
        for (int i = 0; i < FINAL_MSG_TS_LEN; i++)
        {
            ts_field[i] = (uint8)ts;
            ts >>= 8;
        }
    }

    static void GetTimestampFromMessage(const uint8 *ts_field, uint32 *ts)
    {
        *ts = 0;
        for (int i = 0; i < FINAL_MSG_TS_LEN; i++)
        {
            *ts += ts_field[i] << (i * 8);
        }
    }


    //==============================================================================
    // INITIATOR IMPLEMENTATION
    //==============================================================================

    namespace Initiator
    {
        /* Initiator state */
        static bool initialized = false;
        static bool running = false;
        static uint8 frame_seq_nb = 0;
        static volatile bool ranging_in_progress = false;
        static uint32_t ranging_interval_ms = 0;
        static TaskHandle_t ranging_task_handle = NULL;
        static uint32_t ranging_task_priority = 1;

        /* Timestamps */
        static uint64_t poll_tx_ts;
        static uint64_t resp_rx_ts;
        static uint64_t final_tx_ts;

        /* Delays and timeouts */
        constexpr uint16_t POLL_TX_TO_RESP_RX_DLY_UUS = 900;
        constexpr uint16_t RESP_RX_TO_FINAL_TX_DLY_UUS = 1000;
        constexpr uint16_t RESP_RX_TIMEOUT_UUS = 2000;

        /* State machine */
        typedef enum
        {
            STATE_IDLE,
            STATE_POLL_SENT,
            STATE_RESP_RECEIVED,
        } init_state_t;

        static volatile init_state_t current_state = STATE_IDLE;

        static void reset_state()
        {
            current_state = STATE_IDLE;
            ranging_in_progress = false;
            dwt_setrxtimeout(0);
            dwt_forcetrxoff();
        }

        /* Callback handlers */
        static void rx_ok_cb(const dwt_cb_data_t *cb_data)
        {
            dw1000_spi_acquire_bus();

            if (current_state != STATE_POLL_SENT)
            {
                dw1000_spi_release_bus();
                return;
            }

            if (cb_data->datalength <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
                rx_buffer[ALL_MSG_SN_IDX] = 0;

                if (memcmp(rx_buffer, resp_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    current_state = STATE_RESP_RECEIVED;

                    poll_tx_ts = GetTxTimestamp();
                    resp_rx_ts = GetRxTimestamp();

                    uint32 final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(final_tx_time);

                    final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                    SetTimestampInMessage(&final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                    SetTimestampInMessage(&final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                    SetTimestampInMessage(&final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                    final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_writetxdata(sizeof(final_msg), final_msg, 0);
                    dwt_writetxfctrl(sizeof(final_msg), 0, 1);

                    int ret = dwt_starttx(DWT_START_TX_DELAYED);

                    if (ret == DWT_SUCCESS)
                    {
                        frame_seq_nb++;
                        dw1000_spi_release_bus();
#if DEBUG_RANGING
                        Serial.printf("[Init] Response received, final sent\n");
#endif
                        return;
                    }
                    else
                    {
                        reset_state();
                        dw1000_spi_release_bus();
                        return;
                    }
                }
                else
                {
                    reset_state();
                    dw1000_spi_release_bus();
                    return;
                }
            }
            else
            {
                reset_state();
                dw1000_spi_release_bus();
            }
        }

        static void rx_to_cb(const dwt_cb_data_t *cb_data)
        {
            dw1000_spi_acquire_bus();
            reset_state();
            dw1000_spi_release_bus();
        }

        static void rx_err_cb(const dwt_cb_data_t *cb_data)
        {
            dw1000_spi_acquire_bus();
            reset_state();
            dw1000_spi_release_bus();
        }

        static void tx_conf_cb(const dwt_cb_data_t *cb_data)
        {
            if (current_state == STATE_IDLE)
            {
                current_state = STATE_POLL_SENT;
            }
            else
            {
                current_state = STATE_IDLE;
            }

            dw1000_spi_acquire_bus();
            dwt_forcetrxoff();

            if (current_state == STATE_POLL_SENT)
            {
                dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                dwt_rxenable(DWT_START_RX_DELAYED);
            }
            else
            {
                ranging_in_progress = false;
            }
            dw1000_spi_release_bus();
        }

        /* Ranging task for automatic polling */
        static void RangingTask(void *pvParameters)
        {
            while (running)
            {
                if (ranging_interval_ms > 0)
                {
                    TriggerRanging();
                    vTaskDelay(pdMS_TO_TICKS(ranging_interval_ms));
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(100)); // Short delay when interval is 0
                }
            }
            ranging_task_handle = NULL;
            vTaskDelete(NULL);
        }

        bool Initialize(uint32_t callback_priority, uint32_t ranging_priority)
        {
            if (initialized)
            {
                return true;
            }

            /* Store ranging task priority for use in Begin() */
            ranging_task_priority = ranging_priority;

#if DEBUG_INIT
            Serial.println("[Init] Initializing UWB Initiator");
#endif

            /* Setup IMU CS pin to high (deselect IMU) */
            pinMode(IMU_CS_PIN, OUTPUT);
            digitalWrite(IMU_CS_PIN, HIGH);

            /* Configure SPI bus */
            spi_bus_config_t spi_bus_cfg = {
                .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
                .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
                .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
                .max_transfer_sz = 1024,
                .flags = 0,
                .intr_flags = 0};

            if (dw1000_spi_init(SPI_HOST, (gpio_num_t)UWB_CS_PIN, &spi_bus_cfg) != 0)
            {
#if DEBUG_INIT
                Serial.println("[Init] ERROR: SPI init failed!");
#endif
                return false;
            }

            if (dw1000_gpio_init((gpio_num_t)UWB_RST_PIN, (gpio_num_t)UWB_IRQ_PIN, GPIO_NUM_NC) != 0)
            {
#if DEBUG_INIT
                Serial.println("[Init] ERROR: GPIO init failed!");
#endif
                return false;
            }

            dw1000_hard_reset();
            dw1000_spi_fix_bug();
            vTaskDelay(pdMS_TO_TICKS(5));

            if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
            {
#if DEBUG_INIT
                Serial.println("[Init] ERROR: dwt_initialise failed!");
#endif
                return false;
            }

            spi_set_rate_high();

            dwt_configure(&dw1000_config);
            dwt_setrxantennadelay(RX_ANT_DLY);
            dwt_settxantennadelay(TX_ANT_DLY);

            if (dw1000_setup_isr(callback_priority, &tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb) != 0)
            {
#if DEBUG_INIT
                Serial.println("[Init] ERROR: dw1000_setup_isr failed!");
#endif
                return false;
            }

            dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE |
                                 DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXOVRR | DWT_INT_RXPTO |
                                 DWT_INT_SFDT,
                             1);

            dw1000_auto_bus_acquisition(false);

            initialized = true;
#if DEBUG_INIT
            Serial.println("[Init] UWB Initiator initialized successfully");
#endif
            return true;
        }

        bool TriggerRanging()
        {
            if (!initialized)
            {
                return false;
            }

            if (ranging_in_progress)
            {
                return false;
            }

            ranging_in_progress = true;
            current_state = STATE_IDLE;

            dw1000_spi_acquire_bus();
            poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
            dwt_writetxfctrl(sizeof(poll_msg), 0, 1);

            int tx_ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
            dw1000_spi_release_bus();

            if (tx_ret != DWT_SUCCESS)
            {
                ranging_in_progress = false;
                return false;
            }

#if DEBUG_RANGING
            Serial.printf("[Init] Ranging triggered\n");
#endif

            return true;
        }

        void Begin(uint32_t interval_ms)
        {
            if (!initialized || running)
            {
                return;
            }

            ranging_interval_ms = interval_ms;
            running = true;

            /* Create ranging task */
            xTaskCreate(RangingTask, "RangingTask", 2048, NULL, ranging_task_priority, &ranging_task_handle);

#if DEBUG_INIT
            Serial.printf("[Init] Started with interval %u ms\n", interval_ms);
#endif
        }

        void Stop()
        {
            if (!initialized || !running)
            {
                return;
            }

            running = false;
            ranging_interval_ms = 0;

            /* Wait for task to finish */
            while (ranging_task_handle != NULL)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            dw1000_spi_acquire_bus();
            dwt_forcetrxoff();
            dw1000_spi_release_bus();

#if DEBUG_INIT
            Serial.println("[Init] Stopped");
#endif
        }

        bool IsActive()
        {
            return initialized && running;
        }

    } // namespace Initiator

    //==============================================================================
    // RESPONDER IMPLEMENTATION
    //==============================================================================

    namespace Responder
    {

        /* Responder state */
        static bool initialized = false;
        static bool running = false;
        static uint8 frame_seq_nb = 0;
        static QueueHandle_t result_queue = NULL;

        /* Timestamps */
        static uint64_t poll_rx_ts;
        static uint64_t resp_tx_ts;
        static uint64_t final_rx_ts;

        /* Delays and timeouts */
        constexpr uint16_t POLL_RX_TO_RESP_TX_DLY_UUS = 1000;
        constexpr uint16_t RESP_TX_TO_FINAL_RX_DLY_UUS = 900;
        constexpr uint16_t FINAL_RX_TIMEOUT_UUS = 2000;

        /* Queue configuration */
        constexpr uint8_t QUEUE_SIZE = 10;

        /* State machine */
        typedef enum
        {
            STATE_WAITING_POLL,
            STATE_POLL_RECEIVED,
            STATE_RESP_SENT
        } resp_state_t;

        static volatile resp_state_t current_state = STATE_WAITING_POLL;

        static void reset_state()
        {
            current_state = STATE_WAITING_POLL;
            dwt_setrxtimeout(0);
            dwt_forcetrxoff();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* Callback handlers */
        static void rx_ok_cb(const dwt_cb_data_t *cb_data)
        {
            dw1000_spi_acquire_bus();

            if (cb_data->datalength <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
                rx_buffer[ALL_MSG_SN_IDX] = 0;

                if (current_state == STATE_WAITING_POLL && memcmp(rx_buffer, poll_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    current_state = STATE_POLL_RECEIVED;

                    poll_rx_ts = GetRxTimestamp();

                    uint32 resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_writetxdata(sizeof(resp_msg), resp_msg, 0);
                    dwt_writetxfctrl(sizeof(resp_msg), 0, 1);

                    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                    if (ret == DWT_SUCCESS)
                    {
                        frame_seq_nb++;
                        dw1000_spi_release_bus();
                        return;
                    }
                    else
                    {
                        reset_state();
                        dw1000_spi_release_bus();
                        return;
                    }
                }
                else if ((current_state == STATE_RESP_SENT) && memcmp(rx_buffer, final_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32 poll_tx_ts_32, resp_rx_ts_32, final_tx_ts_32;
                    uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                    double Ra, Rb, Da, Db;
                    int64_t tof_dtu;

                    resp_tx_ts = GetTxTimestamp();
                    final_rx_ts = GetRxTimestamp();

                    GetTimestampFromMessage(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts_32);
                    GetTimestampFromMessage(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts_32);
                    GetTimestampFromMessage(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_32);

                    poll_rx_ts_32 = (uint32)poll_rx_ts;
                    resp_tx_ts_32 = (uint32)resp_tx_ts;
                    final_rx_ts_32 = (uint32)final_rx_ts;

                    Ra = (double)(resp_rx_ts_32 - poll_tx_ts_32);
                    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                    Da = (double)(final_tx_ts_32 - resp_rx_ts_32);
                    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);

                    tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                    double tof = tof_dtu * DWT_TIME_UNITS;
                    double distance = tof * SPEED_OF_LIGHT;

                    /* Send result to queue (overwrite oldest if full) */
                    if (result_queue != NULL)
                    {
                        RangingResult result;
                        result.distance_m = distance;
                        result.measurement_time_us = esp_timer_get_time();

                        if (xQueueSend(result_queue, &result, 0) != pdTRUE)
                        {
                            /* Queue full, remove oldest and try again */
                            RangingResult dummy;
                            xQueueReceive(result_queue, &dummy, 0);
                            xQueueSend(result_queue, &result, 0);
                        }
                    }

                    reset_state();
                    dw1000_spi_release_bus();
                    return;
                }
                else
                {
                    reset_state();
                    dw1000_spi_release_bus();
                    return;
                }
            }
            else
            {
                reset_state();
                dw1000_spi_release_bus();
            }
        }

        static void rx_to_cb(const dwt_cb_data_t *cb_data)
        {
            dw1000_spi_acquire_bus();
            reset_state();
            dw1000_spi_release_bus();
        }

        static void rx_err_cb(const dwt_cb_data_t *cb_data)
        {
            dw1000_spi_acquire_bus();
            reset_state();
            dw1000_spi_release_bus();
        }

        static void tx_conf_cb(const dwt_cb_data_t *cb_data)
        {
            current_state = STATE_RESP_SENT;

            dw1000_spi_acquire_bus();
            dwt_forcetrxoff();
            dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
            dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
            dwt_rxenable(DWT_START_RX_DELAYED);
            dw1000_spi_release_bus();
        }

        QueueHandle_t Initialize(uint32_t callback_priority)
        {
            if (initialized)
            {
                return result_queue;
            }

            /* Create internal queue with overwrite capability */
            result_queue = xQueueCreate(QUEUE_SIZE, sizeof(RangingResult));
            if (result_queue == NULL)
            {
                return NULL;
            }

#if DEBUG_INIT
            Serial.println("[Resp] Initializing UWB Responder");
#endif

            /* Setup IMU CS pin to high (deselect IMU) */
            pinMode(IMU_CS_PIN, OUTPUT);
            digitalWrite(IMU_CS_PIN, HIGH);

            /* Configure SPI bus */
            spi_bus_config_t spi_bus_cfg = {
                .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
                .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
                .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
                .max_transfer_sz = 1024,
                .flags = 0,
                .intr_flags = 0};

            if (dw1000_spi_init(SPI_HOST, (gpio_num_t)UWB_CS_PIN, &spi_bus_cfg) != 0)
            {
#if DEBUG_INIT
                Serial.println("[Resp] ERROR: SPI init failed!");
#endif
                vQueueDelete(result_queue);
                result_queue = NULL;
                return NULL;
            }

            if (dw1000_gpio_init((gpio_num_t)UWB_RST_PIN, (gpio_num_t)UWB_IRQ_PIN, GPIO_NUM_NC) != 0)
            {
#if DEBUG_INIT
                Serial.println("[Resp] ERROR: GPIO init failed!");
#endif
                vQueueDelete(result_queue);
                result_queue = NULL;
                return NULL;
            }

            dw1000_hard_reset();
            dw1000_spi_fix_bug();
            vTaskDelay(pdMS_TO_TICKS(5));

            if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
            {
#if DEBUG_INIT
                Serial.println("[Resp] ERROR: dwt_initialise failed!");
#endif
                vQueueDelete(result_queue);
                result_queue = NULL;
                return NULL;
            }

            spi_set_rate_high();

            dwt_configure(&dw1000_config);
            dwt_setrxantennadelay(RX_ANT_DLY);
            dwt_settxantennadelay(TX_ANT_DLY);

            if (dw1000_setup_isr(callback_priority, &tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb) != 0)
            {
#if DEBUG_INIT
                Serial.println("[Resp] ERROR: dw1000_setup_isr failed!");
#endif
                vQueueDelete(result_queue);
                result_queue = NULL;
                return NULL;
            }

            dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE |
                                 DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXOVRR | DWT_INT_RXPTO |
                                 DWT_INT_SFDT,
                             1);

            dw1000_auto_bus_acquisition(false);

            initialized = true;
#if DEBUG_INIT
            Serial.println("[Resp] UWB Responder initialized successfully");
#endif
            return result_queue;
        }

        void Begin()
        {
            if (!initialized || running)
            {
                return;
            }

            reset_state();
            running = true;

#if DEBUG_INIT
            Serial.println("[Resp] Started RX routine");
#endif
        }

        void Stop()
        {
            if (!initialized || !running)
            {
                return;
            }

            running = false;

            dw1000_spi_acquire_bus();
            dwt_forcetrxoff();
            dw1000_spi_release_bus();

#if DEBUG_INIT
            Serial.println("[Resp] Stopped RX routine");
#endif
        }

        bool IsActive()
        {
            return initialized && running;
        }

    } // namespace Responder

} // namespace UWBRanging
