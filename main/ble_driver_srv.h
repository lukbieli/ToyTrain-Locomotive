#ifndef BLE_DRIVER_SRV_H
#define BLE_DRIVER_SRV_H

void ble_driver_srv_task(void);
void update_battery_level(uint8_t bat_level);
void update_battery_voltage(float bat_voltage);

#endif // BLE_DRIVER_SRV_H
