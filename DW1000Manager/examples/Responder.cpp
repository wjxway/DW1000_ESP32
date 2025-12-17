/**
 * @file Responder.cpp
 * @brief UWB DS-TWR Responder Test
 * 
 * This test initializes the DW1000 as a responder and prints
 * ranging statistics every second including update rate, mean,
 * standard deviation, and percentiles.
 */

#include <Arduino.h>
#include <DW1000Manager.hpp>
#include <HardwareDefs.hpp>
#include <Blink.hpp>
#include <driver/spi_master.h>
#include <vector>
#include <algorithm>
#include <cmath>

QueueHandle_t ranging_queue = NULL;

struct Statistics
{
    float mean;
    float std_dev;
    float max_diff;
    uint32_t count;
    float update_rate_hz;
};

Statistics CalculateStatistics(const std::vector<double>& distances, uint32_t time_window_ms)
{
    Statistics stats = {0, 0, 0, 0, 0};
    
    if (distances.empty())
    {
        return stats;
    }
    
    stats.count = distances.size();
    stats.update_rate_hz = (stats.count * 1000.0f) / time_window_ms;
    
    // Calculate mean
    double sum = 0;
    for (double d : distances)
    {
        sum += d;
    }
    stats.mean = sum / stats.count;
    
    // Calculate standard deviation
    double variance_sum = 0;
    for (double d : distances)
    {
        double diff = d - stats.mean;
        variance_sum += diff * diff;
    }
    stats.std_dev = sqrt(variance_sum / (stats.count - 1));
    
    // Calculate max difference (max - min)
    double min_val = *std::min_element(distances.begin(), distances.end());
    double max_val = *std::max_element(distances.begin(), distances.end());
    stats.max_diff = max_val - min_val;
    
    return stats;
}

void setup()
{
    Serial.begin(115200);
    vTaskDelay(1000);
    
    /* Blink LED to indicate start */
    Blink(500, 3, true, true, true);
    
    Serial.println("\n=== UWB DS-TWR Responder Test ===");
    Serial.println("Initializing...");
    
    // this is only necessary on my test board where another IMU is present.
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);
    
    // Initialize SPI bus
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
        .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
        .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0
    };
    
    if (spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK)
    {
        Serial.println("ERROR: Failed to initialize SPI bus!");
        while (1) { vTaskDelay(1000); }
    }
    
    // Initialize the UWB hardware
    ranging_queue = UWBRanging::Responder::Initialize(UWB_CS_PIN, UWB_IRQ_PIN, UWB_RST_PIN);
    
    if (ranging_queue == NULL)
    {
        Serial.println("ERROR: Failed to initialize UWB Responder!");
        while (1)
        {
            vTaskDelay(1000);
        }
    }
    
    Serial.println("UWB Responder initialized successfully");
    
    // Start receiving
    UWBRanging::Responder::Begin();
    
    Serial.println("Responder started and listening...");
    Serial.println("\nFormat: UWB stats | Frequency: XX Hz | Mean: XXX.XXX m | Std: XX.X cm | MaxDiff: XX.X cm");
    Serial.println("------------------------------------------------------------------------------------------\n");
}

void loop()
{
    static std::vector<double> distances;
    static uint32_t last_print_time = millis();
    static uint32_t window_start_time = millis();
    
    uint32_t current_time = millis();
    
    // Receive all available ranging results
    UWBRanging::RangingResult result;
    while (xQueueReceive(ranging_queue, &result, 0) == pdTRUE)
    {
        distances.push_back(result.distance_m);
    }
    
    // Print statistics every second
    if (current_time - last_print_time >= 1000)
    {
        uint32_t time_window = current_time - window_start_time;
        
        if (!distances.empty())
        {
            // Calculate and print statistics
            Statistics stats = CalculateStatistics(distances, time_window);
            
            Serial.printf("UWB stats | Frequency: %.0f Hz | Mean: %.3f m | Std: %.1f cm | MaxDiff: %.1f cm\n",
                          stats.update_rate_hz,
                          stats.mean,
                          stats.std_dev * 100.0f,  // Convert m to cm
                          stats.max_diff * 100.0f); // Convert m to cm
            
            // Clear distances for next window
            distances.clear();
            window_start_time = current_time;
        }
        else
        {
            Serial.println("UWB stats | Frequency: 0 Hz | Mean: --- m | Std: --- cm | MaxDiff: --- cm");
        }
        
        last_print_time = current_time;
    }
    
    // Check if still active
    if (!UWBRanging::Responder::IsActive())
    {
        Serial.println("WARNING: Responder is not active!");
    }
    
    vTaskDelay(10);
}
