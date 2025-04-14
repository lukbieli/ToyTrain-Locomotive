#ifndef BLE_DRIVER_SRV_H
#define BLE_DRIVER_SRV_H

/**
 * @brief Initializes and sets up the BLE driver service.
 * 
 * This function configures the BLE stack, registers the GATT services,
 * and starts advertising.
 */
void BleDriverSrv_Setup(void);

/**
 * @brief Updates the battery level characteristic.
 * 
 * @param bat_level The new battery level as a percentage (0-100%).
 *                  This value will be sent to connected BLE clients.
 */
void BleDriverSrv_UpdateBatteryLevel(uint8_t bat_level);

/**
 * @brief Updates the battery voltage characteristic.
 * 
 * @param bat_voltage The new battery voltage in volts.
 *                    This value will be sent to connected BLE clients.
 */
void BleDriverSrv_UpdateBatteryVoltage(float bat_voltage);

/**
 * @brief Retrieves the current motor speed.
 * 
 * @return The motor speed as a uint8_t value. This value represents
 *         the speed of the motor in an application-defined unit.
 */
uint8_t BleDriverSrv_GetMotorSpeed(void);

/**
 * @brief Retrieves the current motor direction.
 * 
 * @return The motor direction as a uint8_t value. This value indicates
 *         the direction of the motor (e.g., forward or reverse).
 */
uint8_t BleDriverSrv_GetMotorDirection(void);

#endif // BLE_DRIVER_SRV_H
