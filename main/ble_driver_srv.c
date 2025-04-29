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
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "ble_driver_srv.h"

#define ENABLE_LOGI 1
#define ENABLE_LOGD 0
#define ENABLE_LOGE 1
#define ENABLE_LOGBUF 0
#define MY_LOGI(...) do { if (ENABLE_LOGI) ESP_LOGI(__VA_ARGS__); } while(0)
#define MY_LOGD(...) do { if (ENABLE_LOGD) ESP_LOGD(__VA_ARGS__); } while(0)
#define MY_LOGE(...) do { if (ENABLE_LOGE)ESP_LOGE(__VA_ARGS__); } while(0)
#define MY_LOGBUF(...) do { if (ENABLE_LOGBUF) ESP_LOG_BUFFER_HEX(__VA_ARGS__); } while(0)

// Battery Service UUIDs and Handles
#define GATTS_SERVICE_UUID_BATTERY       0x180F
#define GATTS_CHAR_UUID_BATTERY_LEVEL    0x2A19
#define GATTS_CHAR_UUID_BATTERY_VOLTAGE  0x2B18
#define GATTS_CHAR_NUM_BATTERY_LEVEL     0
#define GATTS_CHAR_NUM_BATTERY_VOLTAGE   1
#define GATTS_NUM_HANDLE_BATTERY         8

// Motor Service UUIDs and Handles
#define GATTS_SERVICE_UUID_MOTOR         0x00DD
#define GATTS_CHAR_UUID_MOTOR_SPEED      0xDD01
#define GATTS_CHAR_UUID_MOTOR_DIRECTION  0xDD02
#define GATTS_CHAR_NUM_MOTOR_SPEED       0
#define GATTS_CHAR_NUM_MOTOR_DIRECTION   1
#define GATTS_NUM_HANDLE_MOTOR           8

// General BLE Configuration
#define TEST_MANUFACTURER_DATA_LEN       17
#define GATTS_DEMO_CHAR_VAL_LEN_MAX      0x40
#define PREPARE_BUF_MAX_SIZE             1024

// Advertising Configuration Flags
#define ADV_CONFIG_FLAG                  (1 << 0)
#define SCAN_RSP_CONFIG_FLAG             (1 << 1)

// Profile Identifiers
#define PROFILE_NUM                      2
#define PROFILE_BATTERY_APP_ID           0
#define PROFILE_MOTOR_APP_ID             1

// Logging Tag
#define GATTS_TAG                        "GATTS_DEMO"

// BLE Characteristic Structure
typedef struct {
    uint16_t handle;                     // Handle for the characteristic
    esp_bt_uuid_t uuid;                  // UUID of the characteristic
    esp_gatt_perm_t perm;                // Permissions for the characteristic
    esp_gatt_char_prop_t property;       // Properties of the characteristic
    esp_attr_value_t attr_val;           // Attribute value of the characteristic
    uint16_t descr_handle;               // Handle for the descriptor
    esp_bt_uuid_t descr_uuid;            // UUID of the descriptor
    esp_attr_value_t cccd_val;           // CCCD value for notifications/indications
    bool never_recv;                 // Flag to indicate if the characteristic has been received
} ble_characteristic_t;

// GATT Profile Instance Structure
typedef struct {
    esp_gatts_cb_t gatts_cb;             // Callback function for GATT events
    uint16_t gatts_if;                   // GATT interface
    uint16_t app_id;                     // Application ID
    uint16_t conn_id;                    // Connection ID
    uint16_t service_handle;             // Handle for the service
    esp_gatt_srvc_id_t service_id;       // Service ID
    ble_characteristic_t chars[2];       // Array of characteristics
    uint8_t chars_init_counter;          // Counter for initialized characteristics
    uint8_t numHandle;                   // Number of handles for the service
} gatts_profile_inst_t;

// Local Variables
static float battery_voltage = 3.7f;     // Current battery voltage
static uint8_t battery_level = 97;       // Current battery level percentage
static uint8_t battery_voltage_buf[] = {0x00, 0x00, 0x00, 0x00}; // Buffer for battery voltage
static bool service_connected = false;  // Connection status
static uint8_t motor_speed = 0x00u;     // Current motor speed
static uint8_t motor_direction = 0x00u; // Current motor direction

// CCCD Values for Notifications/Indications
static uint8_t cccd_value_battery_level[2] = {0x00, 0x00}; // Battery Level
static uint8_t cccd_value_battery_voltage[2] = {0x00, 0x00}; // Battery Voltage
static uint8_t cccd_value_motor_speed[2] = {0x00, 0x00}; // Motor Speed
static uint8_t cccd_value_motor_direction[2] = {0x00, 0x00}; // Motor Direction

// Device Name and Advertising Configuration
static char test_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "TTLoc_001";
static uint8_t adv_config_done = 0;

// Advertising Data
#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
// Raw advertising data
static uint8_t raw_adv_data[] = {
    /* Flags */
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,               // Length 2, Data Type ESP_BLE_AD_TYPE_FLAG, Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
    /* TX Power Level */
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB,             // Length 2, Data Type ESP_BLE_AD_TYPE_TX_PWR, Data 2 (-21)
    /* Complete 16-bit Service UUIDs */
    0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xAB, 0xCD    // Length 3, Data Type ESP_BLE_AD_TYPE_16SRV_CMPL, Data 3 (UUID)
};

// Raw scan response data
static uint8_t raw_scan_rsp_data[] = {
    /* Complete Local Name */
    0x0F, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D', 'E', 'M', 'O'   // Length 15, Data Type ESP_BLE_AD_TYPE_NAME_CMPL, Data (ESP_GATTS_DEMO)
};
#else
// Advertising service UUIDs
static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0F, 0x18, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xDD, 0x00, 0x00, 0x00,
};

// Advertising data configuration
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// Scan response data configuration
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

// Advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Local function declarations needed for the BLE driver service
static void gattsProfileBatteryEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gattsProfileMotorEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static gatts_profile_inst_t profile_tab[PROFILE_NUM] = {
    [PROFILE_BATTERY_APP_ID] = {
        .gatts_cb = gattsProfileBatteryEventHandler,
        .gatts_if = ESP_GATT_IF_NONE,
        .app_id = PROFILE_BATTERY_APP_ID,
        .service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0x00,
                .uuid = {
                    .uuid = {.uuid16 = GATTS_SERVICE_UUID_BATTERY},
                    .len = ESP_UUID_LEN_16,
                },
            },
        },
        .chars = {
            [GATTS_CHAR_NUM_BATTERY_LEVEL] = {
                .never_recv = true,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_CHAR_UUID_BATTERY_LEVEL },
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                .attr_val = {
                    .attr_max_len = sizeof(battery_level),
                    .attr_len = sizeof(battery_level),
                    .attr_value = &battery_level,
                },
                .cccd_val = {
                    .attr_max_len = sizeof(cccd_value_battery_level),
                    .attr_len = sizeof(cccd_value_battery_level),
                    .attr_value = cccd_value_battery_level,
                },
                .descr_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG },
                },
            },
            [GATTS_CHAR_NUM_BATTERY_VOLTAGE] = {
                .never_recv = true,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_CHAR_UUID_BATTERY_VOLTAGE },
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                .attr_val = {
                    .attr_max_len = sizeof(battery_voltage_buf),
                    .attr_len = sizeof(battery_voltage_buf),
                    .attr_value = battery_voltage_buf,
                },
                .cccd_val = {
                    .attr_max_len = sizeof(cccd_value_battery_voltage),
                    .attr_len = sizeof(cccd_value_battery_voltage),
                    .attr_value = cccd_value_battery_voltage,
                },
                .descr_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG },
                },
            },
        },
        .chars_init_counter = 0,
        .numHandle = GATTS_NUM_HANDLE_BATTERY,
    },
    [PROFILE_MOTOR_APP_ID] = {
        .gatts_cb = gattsProfileMotorEventHandler,
        .gatts_if = ESP_GATT_IF_NONE,
        .app_id = PROFILE_MOTOR_APP_ID,
        .service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0x00,
                .uuid = {
                    .uuid = {.uuid16 = GATTS_SERVICE_UUID_MOTOR},
                    .len = ESP_UUID_LEN_16,
                },
            },
        },
        .chars = {
            [GATTS_CHAR_NUM_MOTOR_SPEED] = {
                .never_recv = true,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_CHAR_UUID_MOTOR_SPEED },
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_WRITE,
                .attr_val = {
                    .attr_max_len = sizeof(motor_speed),
                    .attr_len = sizeof(motor_speed),
                    .attr_value = &motor_speed,
                },
                .cccd_val = {
                    .attr_max_len = sizeof(cccd_value_motor_speed),
                    .attr_len = sizeof(cccd_value_motor_speed),
                    .attr_value = cccd_value_motor_speed,
                },
                .descr_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG },
                },
            },
            [GATTS_CHAR_NUM_MOTOR_DIRECTION] = {
                .never_recv = true,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_CHAR_UUID_MOTOR_DIRECTION },
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_WRITE,
                .attr_val = {
                    .attr_max_len = sizeof(motor_direction),
                    .attr_len = sizeof(motor_direction),
                    .attr_value = &motor_direction,
                },
                .cccd_val = {
                    .attr_max_len = sizeof(cccd_value_motor_direction),
                    .attr_len = sizeof(cccd_value_motor_direction),
                    .attr_value = cccd_value_motor_direction,
                },
                .descr_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG },
                },
            },
        },
        .chars_init_counter = 0,
        .numHandle = GATTS_NUM_HANDLE_MOTOR,
    },
};

///Declare the static function
static void gattsProfileGenericEventHandler(uint8_t app_id, esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static ble_characteristic_t *findCharacteristicByHandle(gatts_profile_inst_t* profilePtr, uint16_t handle);
static ble_characteristic_t* findCharacteristicByUuid(gatts_profile_inst_t* profilePtr,uint16_t uuid);
static ble_characteristic_t* findCharacteristicByDescriptorHandle(gatts_profile_inst_t* profilePtr,uint16_t handle);
static uint16_t convertUint8ArrToUint16(uint8_t *arr);
static void convertFloatToUint8Arr(uint8_t *buf, float voltage);

// Global Functions
bool BleDriverSrv_IsConnected(void) 
{
    return service_connected;
}

void BleDriverSrv_Setup(void)
{
    MY_LOGI(GATTS_TAG, "BLE Driver Service Setup");
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    #if CONFIG_EXAMPLE_CI_PIPELINE_ID
    memcpy(test_device_name, esp_bluedroid_get_example_name(), ESP_BLE_ADV_NAME_LEN_MAX);
    #endif

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        MY_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        MY_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        MY_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        MY_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gattsEventHandler);
    if (ret){
        MY_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gapEventHandler);
    if (ret){
        MY_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }


    esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name);
    if (set_dev_name_ret){
        MY_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
    }
#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
    esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (raw_adv_ret){
        MY_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
    }
    adv_config_done |= ADV_CONFIG_FLAG;
    esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
    if (raw_scan_ret){
        MY_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
    }
    adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
    //config adv data
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret){
        MY_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
    }
    adv_config_done |= ADV_CONFIG_FLAG;
    //config scan response data
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret){
        MY_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
    }
    adv_config_done |= SCAN_RSP_CONFIG_FLAG;

#endif

    ret = esp_ble_gatts_app_register(PROFILE_BATTERY_APP_ID);
    if (ret) {
        MY_LOGE(GATTS_TAG, "Battery Service app register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_MOTOR_APP_ID);
    if (ret) {
        MY_LOGE(GATTS_TAG, "Motor Service app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        MY_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    MY_LOGI(GATTS_TAG, "BLE Driver Service Setup Complete");


    // //loop
    // while (1) {
    //     vTaskDelay(10000 / portTICK_PERIOD_MS);

    // }
    // return;
}

void BleDriverSrv_UpdateBatteryLevel(uint8_t bat_level) {
    battery_level = bat_level;
    if(service_connected == true) {
        

        // Update the characteristic value
        esp_ble_gatts_set_attr_value(profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_LEVEL].handle, sizeof(battery_level), &battery_level);

        // Send notification if enabled
        if (convertUint8ArrToUint16(profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_LEVEL].cccd_val.attr_value) == 0x0001) {
            
            MY_LOGD(GATTS_TAG, "Sending notification for Battery Level: %d%%", battery_level);
            // Send notification to the connected device
            esp_ble_gatts_send_indicate(profile_tab[PROFILE_BATTERY_APP_ID].gatts_if,
                                        profile_tab[PROFILE_BATTERY_APP_ID].conn_id,
                                        profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_LEVEL].handle,
                                        sizeof(battery_level), &battery_level, false);
        }
    }
    else {
        // MY_LOGD(GATTS_TAG, "Battery Service not connected, skipping update.");
    }

}

void BleDriverSrv_UpdateBatteryVoltage(float bat_voltage) {

    battery_voltage = bat_voltage;

    if(service_connected == true) {
        uint8_t buf[4];
        convertFloatToUint8Arr(buf,battery_voltage);
        // Update the characteristic value
        esp_ble_gatts_set_attr_value(profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_VOLTAGE].handle, sizeof(buf), buf);

        // Send notification if enabled
        if (convertUint8ArrToUint16(profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_VOLTAGE].cccd_val.attr_value) == 0x0001) {
            
            MY_LOGD(GATTS_TAG, "Sending notification for Battery Voltage: %.3f", battery_voltage);
            // Send notification to the connected device
            esp_ble_gatts_send_indicate(profile_tab[PROFILE_BATTERY_APP_ID].gatts_if,
                                        profile_tab[PROFILE_BATTERY_APP_ID].conn_id,
                                        profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_VOLTAGE].handle,
                                        sizeof(buf), buf, false);
        }
    }
    else {
        // MY_LOGD(GATTS_TAG, "Battery Service not connected, skipping update.");
    }
}

bool BleDriverSrv_GetMotorSpeed(uint8_t* val) {

    if (val == NULL) {
        MY_LOGE(GATTS_TAG, "Invalid pointer for motor speed value.");
        return false;
    }
    //check if never recieved
    else if(profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_SPEED].never_recv == true) {
        MY_LOGD(GATTS_TAG, "Motor speed characteristic not received yet.");
        return false;
    }
    else {
        *val = motor_speed;
        return true;
    }
}

bool BleDriverSrv_GetMotorDirection(uint8_t* val) {
    if (val == NULL) {
        MY_LOGE(GATTS_TAG, "Invalid pointer for motor direction value.");
        return false;
    }
    //check if never recieved
    else if(profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_DIRECTION].never_recv == true) {
        MY_LOGD(GATTS_TAG, "Motor direction characteristic not received yet.");
        return false;
    }
    else {
        *val = motor_direction;
        return true;
    }
}

// Local Functions
static void convertFloatToUint8Arr(uint8_t *buf, float voltage)
{
    // Convert float to uint8_t array (4 bytes)
    buf[0] = (uint8_t)((int)(voltage * 1000.0) & 0xFF);         // LSB
    buf[1] = (uint8_t)(((int)(voltage * 1000.0) >> 8) & 0xFF);
    buf[2] = (uint8_t)(((int)(voltage * 1000.0) >> 16) & 0xFF);
    buf[3] = (uint8_t)(((int)(voltage * 1000.0) >> 24) & 0xFF); // MSB
}

static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
            MY_LOGI(GATTS_TAG, "Advertising data set complete, starting advertising...");
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
            MY_LOGI(GATTS_TAG, "Scan response data set complete, starting advertising...");
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            MY_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        MY_LOGD(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            MY_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            break;
        }
        MY_LOGD(GATTS_TAG, "Advertising stop successfully");
        
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         MY_LOGD(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        MY_LOGD(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;

    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
        MY_LOGD(GATTS_TAG, "RSSI: %d dBm", param->read_rssi_cmpl.rssi);
        break;        
    default:
        break;
    }
}

static ble_characteristic_t *findCharacteristicByHandle(gatts_profile_inst_t* profilePtr, uint16_t handle) {
    for (int i = 0; i < 2; i++) {
        if (profilePtr->chars[i].handle == handle) {
            return &profilePtr->chars[i];
        }
    }
    return NULL;
}

static ble_characteristic_t* findCharacteristicByUuid(gatts_profile_inst_t* profilePtr,uint16_t uuid) {
    for (int i = 0; i < GATTS_CHAR_NUM_BATTERY_VOLTAGE+1; i++) {
        if (profilePtr->chars[i].uuid.uuid.uuid16 == uuid) {
            return &profilePtr->chars[i];
        }
    }
    return NULL;
}

static ble_characteristic_t* findCharacteristicByDescriptorHandle(gatts_profile_inst_t* profilePtr,uint16_t handle) {
    for (int i = 0; i < GATTS_CHAR_NUM_BATTERY_VOLTAGE+1; i++) {
        if (profilePtr->chars[i].descr_handle == handle) {
            return &profilePtr->chars[i];
        }
    }
    return NULL;
}

static uint16_t convertUint8ArrToUint16(uint8_t *arr) {
    uint16_t value = 0;
    value |= (arr[1] << 8); // MSB
    value |= arr[0];       // LSB
    return value;
}



//generic handler for all GATT events
static void gattsProfileGenericEventHandler(uint8_t app_id, esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ble_characteristic_t* charPtr = NULL;
    esp_gatt_status_t status = ESP_GATT_OK;
    switch (event) {
    case ESP_GATTS_REG_EVT:
        MY_LOGD(GATTS_TAG, "Service registered Profile%d", app_id);

        esp_ble_gatts_create_service(gatts_if, &profile_tab[app_id].service_id, profile_tab[app_id].numHandle);
        break;

    case ESP_GATTS_CREATE_EVT:
        MY_LOGD(GATTS_TAG, "Service created Profile%d, service_handle %d", app_id, param->create.service_handle);
        profile_tab[app_id].service_handle = param->create.service_handle;

        esp_ble_gatts_start_service(profile_tab[app_id].service_handle);
        
        esp_err_t add_char_ret = esp_ble_gatts_add_char(profile_tab[app_id].service_handle, 
            &(profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].uuid),
            profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].perm,
            profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].property,
            &(profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].attr_val),
            NULL);
        if (add_char_ret) {
            MY_LOGE(GATTS_TAG, "Failed to add characteristic, error code = %x", add_char_ret);
        }
        
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
            MY_LOGD(GATTS_TAG, "Characteristic add Profile%d, status %d, attr_handle %d, service_handle %d",
                     app_id, param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

            charPtr = findCharacteristicByUuid(&profile_tab[app_id], param->add_char.char_uuid.uuid.uuid16);
            if(charPtr != NULL)
            {
                charPtr->handle = param->add_char.attr_handle;
                MY_LOGD(GATTS_TAG, "Characteristic handle: %d assigned to uuid 0x%04x", charPtr->handle, param->add_char.char_uuid.uuid.uuid16);

                if(profile_tab[app_id].chars_init_counter < 2)
                {
                    esp_err_t result = esp_ble_gatts_add_char_descr(profile_tab[app_id].service_handle, &profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].descr_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                &(profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].cccd_val), NULL);
                    if (result) {
                        MY_LOGE(GATTS_TAG, "Failed to add descriptor, error code = %x", result);
                    }
                }
                else {
                    MY_LOGD(GATTS_TAG, "All characteristics and descriptors added add_char_evt.");
                }
            }
            else
            {
                MY_LOGE(GATTS_TAG, "Unknown characteristic UUID 0x%04x", param->add_char.char_uuid.uuid.uuid16);
            }
            

            break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].descr_handle = param->add_char_descr.attr_handle;
        MY_LOGD(GATTS_TAG, "Descriptor add Profile%d, status %d, attr_handle %d, service_handle %d",
                app_id, param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
                    
        if(++profile_tab[app_id].chars_init_counter < 2)
        {
            MY_LOGD(GATTS_TAG, "Adding next characteristic and descriptor...");
            esp_err_t add_char_ret = esp_ble_gatts_add_char(profile_tab[app_id].service_handle, 
                &(profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].uuid),
                profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].perm,
                profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].property,
                &(profile_tab[app_id].chars[profile_tab[app_id].chars_init_counter].attr_val),
                NULL);
            if (add_char_ret) {
                MY_LOGE(GATTS_TAG, "Failed to add characteristic, error code = %x", add_char_ret);
            }
        } else {
            MY_LOGD(GATTS_TAG, "All characteristics and descriptors added add_char_descr_evt.");
        }
        break;

    case ESP_GATTS_START_EVT:
        MY_LOGD(GATTS_TAG, "Service started Profile%d, service_handle %d", app_id, param->start.service_handle);
        break;

    case ESP_GATTS_READ_EVT:
        MY_LOGD(GATTS_TAG, "Characteristic read Profile%d, handle %d", app_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;

        charPtr = findCharacteristicByHandle(&profile_tab[app_id], param->read.handle);
        if(charPtr != NULL)
        {
            rsp.attr_value.len = charPtr->attr_val.attr_len;
            memcpy(rsp.attr_value.value, charPtr->attr_val.attr_value, charPtr->attr_val.attr_len);
        }
        else
        {
            charPtr = findCharacteristicByDescriptorHandle(&profile_tab[app_id], param->read.handle);
            if(charPtr != NULL)
            {
                rsp.attr_value.len = 2; // Length of CCCD value
                memcpy(rsp.attr_value.value, charPtr->cccd_val.attr_value, 2);
            }
            else
            {
                MY_LOGE(GATTS_TAG, "Unknown handle");
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_ERROR, NULL);
                return;
            }
        }
    
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    case ESP_GATTS_WRITE_EVT: 
        MY_LOGD(GATTS_TAG, "Write event Profile%d, handle %d", app_id, param->write.handle);

        charPtr = findCharacteristicByHandle(&profile_tab[app_id], param->write.handle);
        if(charPtr != NULL)
        {
            MY_LOGD(GATTS_TAG, "Characteristic write Profile%d, handle %d", app_id, param->write.handle);
            if (param->write.len > sizeof(charPtr->attr_val.attr_value)) {
                MY_LOGE(GATTS_TAG, "Write value too long");
                status = ESP_GATT_ERROR;
            } else {
                memcpy(charPtr->attr_val.attr_value, param->write.value, param->write.len);
                charPtr->attr_val.attr_len = param->write.len;
                MY_LOGD(GATTS_TAG, "Characteristic value: %d, len: %d", charPtr->attr_val.attr_value[0], charPtr->attr_val.attr_len);
                MY_LOGBUF(GATTS_TAG, charPtr->attr_val.attr_value, charPtr->attr_val.attr_len);
                charPtr->never_recv = false;
            }
        }
        else
        {

            charPtr = findCharacteristicByDescriptorHandle(&profile_tab[app_id],param->write.handle);

            if (charPtr != NULL) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                memcpy(charPtr->cccd_val.attr_value, param->write.value,2);
                MY_LOGD(GATTS_TAG, "Descriptor value: %d", descr_value);
                if (descr_value == 0x0001) {
                    //print as hex in format 0x0001

                    MY_LOGD(GATTS_TAG, "Notifications enabled for char 0x%04x", charPtr->uuid.uuid.uuid16);
                } else if (descr_value == 0x0002) {
                    MY_LOGD(GATTS_TAG, "Indications enabled for  char 0x%04x", charPtr->uuid.uuid.uuid16);
                } else if (descr_value == 0x0000) {
                    MY_LOGD(GATTS_TAG, "Notifications/Indications disabled for char 0x%04x", charPtr->uuid.uuid.uuid16);
                }
            }
            else
            {
                MY_LOGE(GATTS_TAG, "Unknown descriptor handle %d", param->write.handle);
                status = ESP_GATT_ERROR;
            }
        }
        
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        break;
    
    case ESP_GATTS_EXEC_WRITE_EVT:
        MY_LOGD(GATTS_TAG,"Execute write Profile%d", app_id);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
        
    case ESP_GATTS_CONNECT_EVT:
        MY_LOGD(GATTS_TAG, "Connected Profile%d, conn_id %d, remote "ESP_BD_ADDR_STR"",
                app_id, param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        profile_tab[app_id].conn_id = param->connect.conn_id;
        if(service_connected == false)
        {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x30;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x20;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 600;    // timeout = 400*10ms = 4000ms
            MY_LOGD(GATTS_TAG, "Connected A, conn_id %u, remote "ESP_BD_ADDR_STR"",
                     param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);

            MY_LOGI(GATTS_TAG, "Connected to device "ESP_BD_ADDR_STR"",
                    ESP_BD_ADDR_HEX(param->connect.remote_bda));
            
            service_connected = true;
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        MY_LOGD(GATTS_TAG, "Disconnected Profile%d, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                app_id, ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        if(service_connected == true)
        {
            esp_ble_gap_start_advertising(&adv_params);
            MY_LOGI(GATTS_TAG, "Advertising started after disconnection");
        }
        // profile_tab[app_id].conn_id = 0xFF;
        for(int i = 0; i < 2; i++)
        {        
            // memset(profile_tab[app_id].chars[i].attr_val.attr_value,0, profile_tab[app_id].chars[i].attr_val.attr_len);
            memset(profile_tab[app_id].chars[i].cccd_val.attr_value,0, profile_tab[app_id].chars[i].cccd_val.attr_len);
        }
        
        service_connected = false;
        break;

    default:
        break;
    }
}

static void gattsProfileMotorEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    gattsProfileGenericEventHandler(PROFILE_MOTOR_APP_ID, event, gatts_if, param);
}

static void gattsProfileBatteryEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    gattsProfileGenericEventHandler(PROFILE_BATTERY_APP_ID, event, gatts_if, param);
}

static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            MY_LOGD(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == profile_tab[idx].gatts_if) {
                if (profile_tab[idx].gatts_cb) {
                    profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}
