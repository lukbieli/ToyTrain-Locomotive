#include <stdio.h>
#include <driver/adc.h>
#include <esp_log.h>
#include "battery_monitor.h"

#define TAG "BatteryMonitor"

// ADC configuration
#ifdef CONFIG_IDF_TARGET_ESP32
#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_6 // GPIO34 for ESP32-DevKit
#elif CONFIG_IDF_TARGET_ESP32C3
#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_0 // GPIO0 for ESP32-C3
#else
#error "Unsupported target. Please define the target in the CMakeLists.txt file."
#endif


#define ADC_ATTENUATION ADC_ATTEN_DB_11    // Allows measurement up to ~3.6V
#define ADC_WIDTH ADC_WIDTH_BIT_12         // 12-bit resolution
#define BATTERY_VOLTAGE_MAX 4.2f // Maximum battery voltage (Li-ion cell)
#define BATTERY_VOLTAGE_MIN 1.0f // Minimum battery voltage (Li-ion cell)

// Voltage divider ratio (10kΩ/10kΩ)
#define VOLTAGE_DIVIDER_RATIO 2.0

// ADC reference voltage (typically 3.3V for ESP32)
#define ADC_REF_VOLTAGE 3.3

static float battery_voltage = 0.0f; // Variable to store the battery voltage
static uint8_t battery_level = 0;    // Variable to store the battery level percentage

// Function to initialize the ADC
void BatteryMonitor_Init(void)
{
    // Configure ADC width
    adc1_config_width(ADC_WIDTH);

    // Configure ADC channel attenuation
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTENUATION);

    ESP_LOGI(TAG, "Battery voltage measurement initialized.");
}

// Function to read the battery voltage
float BatteryMonitor_Read(void)
{
    // Read raw ADC value
    int raw_adc = adc1_get_raw(BATTERY_ADC_CHANNEL);

    // Convert raw ADC value to voltage
    float measured_voltage = (raw_adc / 4095.0) * ADC_REF_VOLTAGE;

    // Adjust for the voltage divider
    battery_voltage = measured_voltage * VOLTAGE_DIVIDER_RATIO;

    ESP_LOGI(TAG, "Raw ADC: %d, Measured Voltage: %.2fV, Battery Voltage: %.2fV",
             raw_adc, measured_voltage, battery_voltage);

    return battery_voltage;
}

// Function to get the battery level as a percentage
// calculate battery level based on voltage and linear aproximation
// 0% at 3.0V and 100% at 4.2V
uint8_t BatteryMonitor_GetLevel(void)
{

    if (battery_voltage >= BATTERY_VOLTAGE_MAX) {
        battery_level = 100;
    } else if (battery_voltage <= BATTERY_VOLTAGE_MIN) {
        battery_level = 0;
    } else {
        battery_level = (uint8_t)((battery_voltage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100);
    }

    ESP_LOGI(TAG, "Battery Level: %d%%", battery_level);

    return battery_level;
}
