#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_err.h>
#include "ble_driver_srv.h"
#include "motor_driver.h"

//function for freertos task
void locomotive_task(void *arg)
{
    uint8_t battery_level = 100; // Example battery level
    uint8_t motor_speed = 0;
    uint8_t motor_direction = 0;

    
    static uint8_t prev_motor_speed = 0;
    static uint8_t prev_motor_direction = 0;
    while (1) {
        if (BleDriverSrv_IsConnected() == true) {
            
            // // Example: Send data to the BLE characteristic
            // battery_level = (battery_level > 0) ? battery_level-1 : 100; // Simulate battery level decrease
            // BleDriverSrv_UpdateBatteryLevel(battery_level); // Update battery level to 50%
            // // vTaskDelay(5000 / portTICK_PERIOD_MS);
            // BleDriverSrv_UpdateBatteryVoltage((4.2f * (float)battery_level) / 100.0f); // Update battery voltage to 4.1V

            if(BleDriverSrv_GetMotorSpeed(&motor_speed) && BleDriverSrv_GetMotorDirection(&motor_direction)) {
                // Set motor speed and direction based on BLE characteristic values
                MotorDriver_SetSpeed(motor_speed);
                MotorDriver_SetDirection(motor_direction);
                if (motor_speed != prev_motor_speed || motor_direction != prev_motor_direction) {
                    ESP_LOGI("Loc Task", "Motor Speed: %d, Motor Direction: %d", motor_speed, motor_direction);
                    prev_motor_speed = motor_speed;
                    prev_motor_direction = motor_direction;
                }
                // ESP_LOGI("Loc Task", "Motor Speed: %d, Motor Direction: %d", motor_speed, motor_direction);
            }
            // ESP_LOGI("Locomotive Task", "Motor Speed: %d, Motor Direction: %d", motor_speed, motor_direction);
        }

        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void battery_task(void *arg)
{
    uint8_t battery_level = 100; // Example battery level
    while (1) {
        if (BleDriverSrv_IsConnected() == true) {
            // Example: Send data to the BLE characteristic
            battery_level = (battery_level > 0) ? battery_level-1 : 100; // Simulate battery level decrease
            BleDriverSrv_UpdateBatteryLevel(battery_level); // Update battery level to 50%
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Initialize the motor driver
    // This function sets up the necessary GPIO pins and PWM configuration
    MotorDriver_Init();
    ESP_LOGI("Main", "Motor Driver Initialized");

    
    BleDriverSrv_Setup();
    ESP_LOGI("Main", "BLE Driver Service Setup Complete");
    
    // MotorDriver_SetSpeed(50);
    // MotorDriver_SetDirection(0); 

    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // MotorDriver_SetSpeed(20);
    // MotorDriver_SetDirection(1); 
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // MotorDriver_SetSpeed(100);
    // MotorDriver_SetDirection(0); 
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // MotorDriver_Stop();
    // ESP_LOGI("Main", "Motor Stopped");


    //create task for locomotive
    xTaskCreate(locomotive_task, "locomotive_task", 2048, NULL, 5, NULL);

    //create task for battery
    xTaskCreate(battery_task, "battery_task", 2048, NULL, 5, NULL);

}
