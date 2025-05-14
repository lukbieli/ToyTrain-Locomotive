#include "light_driver.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include <stdio.h> // For debugging/logging if needed

#define TAG "LIGHT_DRV"
// Define GPIO pins for front and back lights
#define LIGHT_FRONT_GPIO 1 // Example GPIO pin for front light
#define LIGHT_BACK_GPIO 3  // Example GPIO pin for back light
#define LIGHT_NUMBER 2 // Number of lights


// define typedef struct for generic light driver
typedef struct {
    int gpio_num; // GPIO number for the light
    int state;    // State of the light (0 = off, 1 = on)
    int blink_interval; // Blink interval in milliseconds
    bool blinking; // Flag to indicate if the light is blinking
    int blink_timer; // Timer for blinking  
} light_driver_t;

// Static variables for light driver
static int light_task_periodicy = 0; // Task periodicy for the light driver
static light_driver_t light_driver[2] = {
    {LIGHT_FRONT_GPIO, 0, 0, false, 0}, // Front light
    {LIGHT_BACK_GPIO, 0, 0, false, 0}   // Back light
};

void LightDriver_Init(int task_periodicy) {

    // Set the task priority for the light driver
    light_task_periodicy = task_periodicy;

    // Initialize the GPIO pins for the lights
    ESP_LOGI(TAG, "Initializing Light Driver");
    uint64_t light_gpio_mask = 0;
    //prepare bitmask for GPIO
    for(int i = 0; i < LIGHT_NUMBER; i++) {
        light_gpio_mask |= (1ULL << light_driver[i].gpio_num);
    }

    // Initialize hardware or GPIO pins for the lights
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = light_gpio_mask,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Set initial state of lights to off
    for(int i = 0; i < LIGHT_NUMBER; i++) {
        gpio_set_level(light_driver[i].gpio_num, 0); // Set initial state to off
    }
    
    ESP_LOGI(TAG,"LightDriver initialized.\n");
}

void LightDriver_Task(void) {
    for(int i = 0; i < LIGHT_NUMBER; i++) {
        if(light_driver[i].blinking) {
            light_driver[i].blink_timer += light_task_periodicy;
            if(light_driver[i].blink_timer >= light_driver[i].blink_interval) {
                light_driver[i].blink_timer = 0;
                // Toggle the light state
                light_driver[i].state ^= 1; // Toggle state (0 -> 1 or 1 -> 0)
                gpio_set_level(light_driver[i].gpio_num, light_driver[i].state);
            }
        }
    }
}

void LightDriver_TurnOn(light_state_t light) {
    if(light >= 0 && light < LIGHT_NUMBER) {
        light_driver[light].state = 1;
        gpio_set_level(light_driver[light].gpio_num, 1); // Turn on the light
        light_driver[light].blinking = false; // Stop blinking if turning on
    } else {
        ESP_LOGE(TAG, "Invalid light state: %d", light);
    }
}

void LightDriver_TurnOff(light_state_t light) {
    if(light >= 0 && light < LIGHT_NUMBER) {
        light_driver[light].state = 0;
        gpio_set_level(light_driver[light].gpio_num, 0); // Turn off the light
        light_driver[light].blinking = false; // Stop blinking if turning off
    } else {
        ESP_LOGE(TAG, "Invalid light state: %d", light);
    }
}

void LightDriver_Blink(light_state_t light, int blink_interval) {
    if(light >= 0 && light < LIGHT_NUMBER) {
        light_driver[light].blink_interval = blink_interval;
        light_driver[light].blinking = true; // Set blinking flag
        light_driver[light].blink_timer = 0; // Reset blink timer
        gpio_set_level(light_driver[light].gpio_num, 0); // Turn off the light
    } else {
        ESP_LOGE(TAG, "Invalid light state: %d", light);
    }
}
