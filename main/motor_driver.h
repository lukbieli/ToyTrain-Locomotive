#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/**
 * @brief Initializes the motor driver.
 * 
 * This function sets up the necessary GPIO pins and PWM configuration
 * for controlling the motor.
 * 
 * @return void
 */
void MotorDriver_Init(void);

/**
 * @brief Sets the speed of the motor.
 * 
 * @param speed The desired speed of the motor (0-100%).
 *              0% means stop, 100% means full speed.
 * *              The speed is set using PWM.
 * @return void
 */
void MotorDriver_SetSpeed(uint8_t speed); // Speed range: 0-100%

/**
 * @brief Sets the direction of the motor.
 * 
 * @param direction The desired direction of the motor (0 for forward, 1 for backward).
 * @return void
 */
void MotorDriver_SetDirection(uint8_t direction); // Direction: 0 for forward, 1 for backward

/**
 * @brief Stops the motor.
 * 
 * This function stops the motor by setting the PWM duty cycle to 0.
 * 
 * @return void
 */
void MotorDriver_Stop(void);


#endif // MOTOR_DRIVER_H
