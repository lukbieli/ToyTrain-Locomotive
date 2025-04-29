#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdio.h>
#include <stdint.h>

/**
 * @brief Initializes the ADC for battery voltage measurement.
 */
void BatteryMonitor_Init(void);

/**
 * @brief Reads the battery voltage from the ADC.
 * 
 * @return The battery voltage in volts.
 *         The value is calculated based on the ADC reading and the voltage divider ratio.
 */
float BatteryMonitor_Read(void);

/**
 * @brief Battery lever percentage.
 * 
 * * @return The battery level as a percentage (0-100%).
 */
uint8_t BatteryMonitor_GetLevel(void);

#endif // BATTERY_MONITOR_H
