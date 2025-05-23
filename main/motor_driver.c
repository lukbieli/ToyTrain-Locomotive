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

#include "motor_driver.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/mcpwm.h"
/* pinout for ESP32-Dev */
#define PWM_GPIO             18        // GPIO connects to the PWM signal line
#define MOT_IN1_GPIO        19        // GPIO connects to the motor driver IN1 pin
#define MOT_IN2_GPIO        5        // GPIO connects to the motor driver IN2 pin
#define MOT_STBY_GPIO      21        // GPIO connects to the motor driver STBY pin

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        1000    // 1000 ticks, 1ms
#define SERVO_MIN_PULSEWIDTH_US 0  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US SERVO_TIMEBASE_PERIOD  // Maximum pulse width in microsecond
#elif CONFIG_IDF_TARGET_ESP32C3
#include "driver/ledc.h"
/* pinout for ESP32-C3 */
#define PWM_GPIO             10        // GPIO connects to the PWM signal line
#define MOT_IN1_GPIO        6        // GPIO connects to the motor driver IN1 pin
#define MOT_IN2_GPIO        7        // GPIO connects to the motor driver IN2 pin
#define MOT_STBY_GPIO      5        // GPIO connects to the motor driver STBY pin

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX               (8192) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

#else
#error "Unsupported target. Please define the target in the CMakeLists.txt file."
#endif



static const char *TAG = "MOT_DRV";
// static mcpwm_cmpr_handle_t comparator = NULL;

/**
 * @brief Initializes the motor driver.
 * 
 * This function sets up the necessary GPIO pins and PWM configuration
 * for controlling the motor.
 * 
 * @return void
 */
void MotorDriver_Init(void)
{
    ESP_LOGI(TAG, "Initializing Motor Driver");
    // Set up GPIO pins for motor control
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOT_IN1_GPIO) | (1ULL << MOT_IN2_GPIO) | (1ULL << MOT_STBY_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "GPIO pins configured for motor control");

    // Set the standby pin to high to enable the motor driver
    ESP_ERROR_CHECK(gpio_set_level(MOT_STBY_GPIO, 1));

    //set the in pins to low to stop the motor
    ESP_ERROR_CHECK(gpio_set_level(MOT_IN1_GPIO, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOT_IN2_GPIO, 0));
    ESP_LOGI(TAG, "Motor driver enabled and stopped");
    
#ifdef CONFIG_IDF_TARGET_ESP32
    // initialize PWM output for motor control on pin 18
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, // esp32 default clock source
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = PWM_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
#elif CONFIG_IDF_TARGET_ESP32C3
    // Initialize the LEDC peripheral for PWM control
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_GPIO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

#else
    #error "Unsupported target. Please define the target in the CMakeLists.txt file."
#endif

}

/**
 * @brief Sets the speed of the motor.
 * 
 * @param speed The desired speed of the motor (0-100%).
 *              0% means stop, 100% means full speed.
 * *              The speed is set using PWM.
 * @return void
 */
void MotorDriver_SetSpeed(uint8_t speed)
{
    if (speed > 100) {
        speed = 100; // Limit speed to 100%
    }
#ifdef CONFIG_IDF_TARGET_ESP32
    // Set the PWM duty cycle based on the speed (0-100%)
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US + ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * speed) / 100;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_width));
    ESP_LOGI(TAG, "Motor speed set to %d, pulse_width: %lu", speed, pulse_width);
#elif CONFIG_IDF_TARGET_ESP32C3
    // Set the PWM duty cycle based on the speed (0-100%)
    uint32_t duty = (LEDC_DUTY_MAX * speed) / 100; // Calculate duty cycle based on speed (0-100%)
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty)); // Set the duty cycle
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL)); // Update the duty cycle  
#else
    #error "Unsupported target. Please define the target in the CMakeLists.txt file."
#endif
}

/**
 * @brief Sets the direction of the motor.
 * 
 * @param direction The desired direction of the motor (0 for forward, 1 for backward).
 * @return void
 */
void MotorDriver_SetDirection(uint8_t direction)
{
    if (direction == 0) {
        // Set motor direction to forward / CW
        ESP_ERROR_CHECK(gpio_set_level(MOT_IN1_GPIO, 1));
        ESP_ERROR_CHECK(gpio_set_level(MOT_IN2_GPIO, 0));
    } else {
        // Set motor direction to backward / CCW
        ESP_ERROR_CHECK(gpio_set_level(MOT_IN1_GPIO, 0));
        ESP_ERROR_CHECK(gpio_set_level(MOT_IN2_GPIO, 1));
    }
    // ESP_LOGI(TAG, "Motor direction set to %s", (direction == 0) ? "forward" : "backward");
}


/**
 * @brief Stops the motor.
 * 
 * This function stops the motor by setting the PWM duty cycle to 0.
 * 
 * @return void
 */
void MotorDriver_Stop(void)
{
    // Set the PWM duty cycle to 0% to stop the motors
    MotorDriver_SetSpeed(0);

    ESP_ERROR_CHECK(gpio_set_level(MOT_IN1_GPIO, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOT_IN2_GPIO, 0));
    // ESP_LOGI(TAG, "Motor stopped");
}
