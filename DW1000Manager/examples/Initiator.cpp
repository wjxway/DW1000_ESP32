/**
 * @file Initiator.cpp
 * @brief UWB DS-TWR Initiator Test
 * 
 * This test initializes the DW1000 as an initiator and triggers
 * ranging exchanges every 20ms.
 */

#include <Arduino.h>
#include <DW1000Manager.hpp>
#include <Blink.hpp>

void setup()
{
    Serial.begin(115200);
    delay(1000);

    /* Blink LED to indicate start */
    Blink(500, 3, true, true, true);
    
    Serial.println("\n=== UWB DS-TWR Initiator Test ===");
    Serial.println("Initializing...");
    
    // Initialize the UWB hardware
    if (!UWBRanging::Initiator::Initialize())
    {
        Serial.println("ERROR: Failed to initialize UWB Initiator!");
        while (1)
        {
            delay(1000);
        }
    }
    
    Serial.println("UWB Initiator initialized successfully");
    
    // Start ranging with 20ms interval
    UWBRanging::Initiator::Begin(20);
    
    Serial.println("Ranging started with 20ms interval");
    Serial.println("Initiator is now sending ranging polls...\n");
}

void loop()
{
    // Check if still active
    if (UWBRanging::Initiator::IsActive())
    {
        static uint32_t last_status_time = 0;
        uint32_t current_time = millis();
        
        // Print status every 5 seconds
        if (current_time - last_status_time >= 5000)
        {
            Serial.println("Initiator active and ranging...");
            last_status_time = current_time;
        }
    }
    else
    {
        Serial.println("WARNING: Initiator is not active!");
    }
    
    delay(100);
}
