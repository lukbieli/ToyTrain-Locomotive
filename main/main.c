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
    while (1) {
        // Your task code here
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        // Example: Send data to the BLE characteristic
        update_battery_level((--battery_level > 0) ? battery_level : 0 ); // Update battery level to 50%
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        update_battery_voltage((4.2f * (float)battery_level) / 100.0f); // Update battery voltage to 4.1V
        uint8_t motor_speed = read_motor_speed();
        uint8_t motor_direction = read_motor_direction();
        ESP_LOGI("Locomotive Task", "Motor Speed: %d, Motor Direction: %d", motor_speed, motor_direction);
    }
}

void app_main(void)
{
    //create task for locomotive
    xTaskCreate(locomotive_task, "locomotive_task", 2048, NULL, 5, NULL);

    ble_driver_srv_task();
}
