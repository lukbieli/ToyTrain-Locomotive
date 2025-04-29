/*
 * Copyright 2025 Lukasz Bielinski
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

//stete machine for the task
typedef enum {
    LOCOMOTIVE_IDLE,
    LOCOMOTIVE_RUNNING,
    LOCOMOTIVE_ERROR
} locomotive_state_t;

static locomotive_state_t locomotive_state = LOCOMOTIVE_IDLE;

static locomotive_state_t state_running(void)
{
    uint8_t motor_speed = 0;
    uint8_t motor_direction = 0;
    static uint8_t prev_motor_speed = 0;
    static uint8_t prev_motor_direction = 0;
    if(BleDriverSrv_GetMotorSpeed(&motor_speed) && BleDriverSrv_GetMotorDirection(&motor_direction)) {
        // Set motor speed and direction based on BLE characteristic values
        if(motor_speed == 0)
        {
            MotorDriver_Stop();
        }
        else
        {
            MotorDriver_SetSpeed(motor_speed);
            MotorDriver_SetDirection(motor_direction);
        }
        if (motor_speed != prev_motor_speed || motor_direction != prev_motor_direction) {
            ESP_LOGI("Loc Task", "Motor Speed: %d, Motor Direction: %d", motor_speed, motor_direction);
            prev_motor_speed = motor_speed;
            prev_motor_direction = motor_direction;
        }
    }

    return LOCOMOTIVE_RUNNING;
}

static locomotive_state_t state_machine(locomotive_state_t state)
{
    switch (state) {
        case LOCOMOTIVE_IDLE:
            // Perform actions for idle state
            if (BleDriverSrv_IsConnected() == true) {
                ESP_LOGI("Locomotive Task", "Locomotive is connected. Moving to RUNNING state");
                state = LOCOMOTIVE_RUNNING;
            }
            break;
        case LOCOMOTIVE_RUNNING:
            // Perform actions for running state
            state = state_running();
            if (BleDriverSrv_IsConnected() == false) {
                ESP_LOGI("Locomotive Task", "Locomotive is disconnected. Moving to IDLE state");
                MotorDriver_Stop(); 
                state = LOCOMOTIVE_IDLE;
            }
            break;
        case LOCOMOTIVE_ERROR:
            // Handle error state
            MotorDriver_Stop();
            break;
    }
    return state;
}

//function for freertos task
void locomotive_task(void *arg)
{
    uint8_t battery_level = 100; // Example battery level
    uint8_t motor_speed = 0;
    uint8_t motor_direction = 0;

    
    static uint8_t prev_motor_speed = 0;
    static uint8_t prev_motor_direction = 0;
    while (1) {
        //call state machine
        locomotive_state = state_machine(locomotive_state);

        
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
