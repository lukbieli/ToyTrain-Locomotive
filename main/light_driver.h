#ifndef LIGHT_DRIVER_H
#define LIGHT_DRIVER_H

#include <stdio.h>
#include <stdint.h>

typedef enum {
    LIGHT_FRONT = 0,
    LIGHT_BACK = 1
} light_state_t;

/**
 * @brief Initializes the light driver.
 * 
 * This function sets up the necessary GPIO pins for controlling the lights.
 * 
 * @param task_periodicy The task periodicy for the light driver.
 * 
 * @return void
 */
void LightDriver_Init(int task_periodicy);

/**
 * @brief Task function for the light driver.
 * 
 * This function should be called periodically to handle light operations.
 * Periodicy is set in the LightDriver_Init function.
 * 
 * @return void
 */
void LightDriver_Task(void);

/**
 * @brief Turns on the specified light.
 * 
 * @param light The light to turn on (LIGHT_FRONT or LIGHT_BACK).
 * 
 * @return void
 */
void LightDriver_TurnOn(light_state_t light);

/**
 * @brief Turns off the specified light.
 * 
 * @param light The light to turn off (LIGHT_FRONT or LIGHT_BACK).
 * 
 * @return void
 */
void LightDriver_TurnOff(light_state_t light);

/**
 * @brief Blinks the specified light.
 * 
 * @param light The light to blink (LIGHT_FRONT or LIGHT_BACK).
 * @param blink_interval The interval in milliseconds for blinking.
 * 
 * @return void
 */
void LightDriver_Blink(light_state_t light, int blink_interval);

#endif // LIGHT_DRIVER_H
