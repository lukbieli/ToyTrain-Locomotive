#ifndef BLE_DRIVER_SRV_H
#define BLE_DRIVER_SRV_H

void ble_driver_srv_task(void);
void update_battery_level(uint8_t bat_level);
void update_battery_voltage(float bat_voltage);
uint8_t read_motor_speed(void);
uint8_t read_motor_direction(void);

#endif // BLE_DRIVER_SRV_H
