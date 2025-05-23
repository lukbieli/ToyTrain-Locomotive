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
#include "battery_monitor.h"
#include "driver/gpio.h"
#include "light_driver.h"

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
    static uint8_t prev_motor_speed = 0xFF;
    static uint8_t prev_motor_direction = 0xFF;
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
        if ((motor_speed != prev_motor_speed) || (motor_direction != prev_motor_direction)) {
            ESP_LOGI("Loc Task", "Motor Speed: %d, Motor Direction: %d", motor_speed, motor_direction);
            prev_motor_speed = motor_speed;
            prev_motor_direction = motor_direction;
            // Update the light state based on motor speed
            if((motor_speed > 0)) {
                LightDriver_TurnOn(LIGHT_FRONT);
                LightDriver_TurnOff(LIGHT_BACK);
            } else {
                LightDriver_TurnOff(LIGHT_FRONT);
                LightDriver_TurnOn(LIGHT_BACK);
            }
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
                LightDriver_TurnOff(LIGHT_FRONT);
                LightDriver_Blink(LIGHT_BACK,200);
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

    LightDriver_Blink(LIGHT_BACK,200);
    
    static uint8_t prev_motor_speed = 0;
    static uint8_t prev_motor_direction = 0;
    while (1) {
        //call state machine
        locomotive_state = state_machine(locomotive_state);

        // call light driver task
        LightDriver_Task();
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void battery_task(void *arg)
{
    uint8_t battery_level = 100;
    // Initialize the battery monitor
    BatteryMonitor_Init();
    while (1) {
        // read battery voltage
        float bat_volt = BatteryMonitor_Read();
        battery_level = BatteryMonitor_GetLevel();
        if (BleDriverSrv_IsConnected() == true) {
            
            
            BleDriverSrv_UpdateBatteryLevel(battery_level); 
            BleDriverSrv_UpdateBatteryVoltage(bat_volt);
        }
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Initialize the motor driver
    // This function sets up the necessary GPIO pins and PWM configuration
    MotorDriver_Init();
    ESP_LOGI("Main", "Motor Driver Initialized");

    // Initialize the light driver
    // This function sets up the necessary GPIO pins for controlling the lights
    LightDriver_Init(50);
    ESP_LOGI("Main", "Light Driver Initialized");

    //sleeep for 10 second
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Initialize the BLE driver service
    BleDriverSrv_Setup();
    ESP_LOGI("Main", "BLE Driver Service Setup Complete");

    //create task for locomotive
    xTaskCreate(locomotive_task, "locomotive_task", 2048 * 2, NULL, 5, NULL);

    //create task for battery
    xTaskCreate(battery_task, "battery_task", 2048, NULL, 5, NULL);

}
