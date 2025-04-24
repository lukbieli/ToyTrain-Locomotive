#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_err.h>
#include "ble_driver_srv.h"

//function for freertos task
void locomotive_task(void *arg)
{
    uint8_t battery_level = 100; // Example battery level
    uint8_t motor_speed = 0;
    uint8_t motor_direction = 0;
    while (1) {
        if (BleDriverSrv_IsConnected() == true) {
            
            // Example: Send data to the BLE characteristic
            battery_level = (battery_level > 0) ? battery_level-1 : 100; // Simulate battery level decrease
            BleDriverSrv_UpdateBatteryLevel(battery_level); // Update battery level to 50%
            // vTaskDelay(5000 / portTICK_PERIOD_MS);
            BleDriverSrv_UpdateBatteryVoltage((4.2f * (float)battery_level) / 100.0f); // Update battery voltage to 4.1V

            if(BleDriverSrv_GetMotorSpeed(&motor_speed) == false) {
                ESP_LOGE("Locomotive Task", "Failed to get motor speed");
            }
            if(BleDriverSrv_GetMotorDirection(&motor_direction) == false) {
                ESP_LOGE("Locomotive Task", "Failed to get motor direction");
            }
            ESP_LOGI("Locomotive Task", "Motor Speed: %d, Motor Direction: %d", motor_speed, motor_direction);
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    //create task for locomotive
    xTaskCreate(locomotive_task, "locomotive_task", 2048, NULL, 5, NULL);

    BleDriverSrv_Setup();
}
