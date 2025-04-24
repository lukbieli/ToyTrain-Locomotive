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
 * @param val Pointer to a uint8_t variable where the motor speed
 *            will be stored.
 * 
 * @return true if the motor speed was successfully retrieved,
 *        false if the pointer is invalid or the characteristic has not been received yet.
 */
bool BleDriverSrv_GetMotorSpeed(uint8_t* val);

/**
 * @brief Retrieves the current motor direction.
 * 
 * @param val Pointer to a uint8_t variable where the motor direction
 * 
 * @return true if the motor direction was successfully retrieved,
 *         false if the pointer is invalid or the characteristic has not been received yet.
 */
bool BleDriverSrv_GetMotorDirection(uint8_t* val);

/**
 * @brief Indicates if connection to the BLE service is established.
 * 
 * @return true if connected, false otherwise.
 */
bool BleDriverSrv_IsConnected(void);

#endif // BLE_DRIVER_SRV_H
