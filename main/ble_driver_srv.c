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


float battery_voltage = 3.7f;
uint8_t battery_level = 97;
uint8_t batteryVoltageBuf[] = {0x00, 0x00, 0x00, 0x00};
bool serviceConnected = false;
uint8_t motorSpeed = 0x00u;
uint8_t motorDirection = 0x00u;

 // CCCD for Battery Level
uint8_t cccd_value_battery_level[2] = {0x00, 0x00}; // Default value: notifications/indications disabled

// CCCD for Battery Voltage
uint8_t cccd_value_battery_voltage[2] = {0x00, 0x00}; // Default value: notifications/indications disabled

// CCCD for motor speed
uint8_t cccd_value_motor_speed[2] = {0x00, 0x00}; // Default value: notifications/indications disabled

// CCCD for motor direction
uint8_t cccd_value_motor_direction[2] = {0x00, 0x00}; // Default value: notifications/indications disabled

#define GATTS_TAG "GATTS_DEMO"

///Declare the static function
static void gatts_profile_generic_event_handler(uint8_t app_id, esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_battery_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_motor_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void convertFloatToUint8Buf(uint8_t *buf, float voltage);


//Service Battery
#define GATTS_SERVICE_UUID_BATTERY   0x180F
#define GATTS_CHAR_UUID_BATTERY_LEVEL  0x2A19
#define GATTS_CHAR_UUID_BATTERY_VOLTAGE 0x2B18
#define GATTS_CHAR_NUM_BATTERY_LEVEL 0
#define GATTS_CHAR_NUM_BATTERY_VOLTAGE 1
#define GATTS_NUM_HANDLE_BATTERY     8

//Service motor
#define GATTS_SERVICE_UUID_MOTOR   0x00DD
#define GATTS_CHAR_UUID_MOTOR_SPEED  0xDD01
#define GATTS_CHAR_UUID_MOTOR_DIRECTION 0xDD02
#define GATTS_CHAR_NUM_MOTOR_SPEED 0
#define GATTS_CHAR_NUM_MOTOR_DIRECTION 1
#define GATTS_NUM_HANDLE_MOTOR     8

static char test_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "ESP_GATTS_DEMO";

#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* Flags */
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,               // Length 2, Data Type ESP_BLE_AD_TYPE_FLAG, Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
    /* TX Power Level */
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB,             // Length 2, Data Type ESP_BLE_AD_TYPE_TX_PWR, Data 2 (-21)
    /* Complete 16-bit Service UUIDs */
    0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xAB, 0xCD    // Length 3, Data Type ESP_BLE_AD_TYPE_16SRV_CMPL, Data 3 (UUID)
};

static uint8_t raw_scan_rsp_data[] = {
    /* Complete Local Name */
    0x0F, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D', 'E', 'M', 'O'   // Length 15, Data Type ESP_BLE_AD_TYPE_NAME_CMPL, Data (ESP_GATTS_DEMO)
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0F, 0x18, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xDD, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
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
// scan response data
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

#define PROFILE_NUM 2
#define PROFILE_BATTERY_APP_ID 0
#define PROFILE_MOTOR_APP_ID 1

typedef struct  {
    uint16_t handle;
    esp_bt_uuid_t uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    esp_attr_value_t attr_val;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
    esp_attr_value_t cccd_val;
} ble_characteristic_t;

typedef struct {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    ble_characteristic_t chars[2]; // Array of characteristics
    uint8_t chars_init_counter;
    uint8_t numHandle;
}gatts_profile_inst_t;

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static gatts_profile_inst_t gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_BATTERY_APP_ID] = {
        .gatts_cb = gatts_profile_battery_event_handler,
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
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_CHAR_UUID_BATTERY_VOLTAGE },
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                .attr_val = {
                    .attr_max_len = sizeof(batteryVoltageBuf),
                    .attr_len = sizeof(batteryVoltageBuf),
                    .attr_value = batteryVoltageBuf,
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
        .gatts_cb = gatts_profile_motor_event_handler,
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
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_CHAR_UUID_MOTOR_SPEED },
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_WRITE,
                .attr_val = {
                    .attr_max_len = sizeof(motorSpeed),
                    .attr_len = sizeof(motorSpeed),
                    .attr_value = &motorSpeed,
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
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_CHAR_UUID_MOTOR_DIRECTION },
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_WRITE,
                .attr_val = {
                    .attr_max_len = sizeof(motorDirection),
                    .attr_len = sizeof(motorDirection),
                    .attr_value = &motorDirection,
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


static void convertFloatToUint8Buf(uint8_t *buf, float voltage)
{
    // Convert float to uint8_t array (4 bytes)
    buf[0] = (uint8_t)((int)(voltage * 1000) & 0xFF);         // LSB
    buf[1] = (uint8_t)(((int)(voltage * 1000) >> 8) & 0xFF);
    buf[2] = (uint8_t)(((int)(voltage * 1000) >> 16) & 0xFF);
    buf[3] = (uint8_t)(((int)(voltage * 1000) >> 24) & 0xFF); // MSB
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
        
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;

    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "RSSI: %d dBm", param->read_rssi_cmpl.rssi);
        break;        
    default:
        break;
    }
}

static ble_characteristic_t *get_characteristic_by_handle(gatts_profile_inst_t* profilePtr, uint16_t handle) {
    for (int i = 0; i < 2; i++) {
        if (profilePtr->chars[i].handle == handle) {
            return &profilePtr->chars[i];
        }
    }
    return NULL;
}

static ble_characteristic_t* get_characteristic_by_uuid(gatts_profile_inst_t* profilePtr,uint16_t uuid) {
    for (int i = 0; i < GATTS_CHAR_NUM_BATTERY_VOLTAGE+1; i++) {
        if (profilePtr->chars[i].uuid.uuid.uuid16 == uuid) {
            return &profilePtr->chars[i];
        }
    }
    return NULL;
}

static ble_characteristic_t* get_characteristic_by_descriptor_handle(gatts_profile_inst_t* profilePtr,uint16_t handle) {
    for (int i = 0; i < GATTS_CHAR_NUM_BATTERY_VOLTAGE+1; i++) {
        if (profilePtr->chars[i].descr_handle == handle) {
            return &profilePtr->chars[i];
        }
    }
    return NULL;
}

static uint16_t convertArrToUint16(uint8_t *arr) {
    uint16_t value = 0;
    value |= (arr[1] << 8); // MSB
    value |= arr[0];       // LSB
    return value;
}



//generic handler for all GATT events
static void gatts_profile_generic_event_handler(uint8_t app_id, esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ble_characteristic_t* charPtr = NULL;
    esp_gatt_status_t status = ESP_GATT_OK;
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "Service registered Profile%d", app_id);

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[app_id].service_id, gl_profile_tab[app_id].numHandle);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "Service created Profile%d, service_handle %d", app_id, param->create.service_handle);
        gl_profile_tab[app_id].service_handle = param->create.service_handle;

        esp_ble_gatts_start_service(gl_profile_tab[app_id].service_handle);
        
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[app_id].service_handle, 
            &(gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].uuid),
            gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].perm,
            gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].property,
            &(gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].attr_val),
            NULL);
        if (add_char_ret) {
            ESP_LOGE(GATTS_TAG, "Failed to add characteristic, error code = %x", add_char_ret);
        }
        
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(GATTS_TAG, "Characteristic add Profile%d, status %d, attr_handle %d, service_handle %d",
                     app_id, param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

            charPtr = get_characteristic_by_uuid(&gl_profile_tab[app_id], param->add_char.char_uuid.uuid.uuid16);
            if(charPtr != NULL)
            {
                charPtr->handle = param->add_char.attr_handle;
                ESP_LOGI(GATTS_TAG, "Characteristic handle: %d assigned to uuid 0x%04x", charPtr->handle, param->add_char.char_uuid.uuid.uuid16);

                if(gl_profile_tab[app_id].chars_init_counter < 2)
                {
                    esp_err_t result = esp_ble_gatts_add_char_descr(gl_profile_tab[app_id].service_handle, &gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].descr_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                &(gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].cccd_val), NULL);
                    if (result) {
                        ESP_LOGE(GATTS_TAG, "Failed to add descriptor, error code = %x", result);
                    }
                }
                else {
                    ESP_LOGI(GATTS_TAG, "All characteristics and descriptors added add_char_evt.");
                }
            }
            else
            {
                ESP_LOGE(GATTS_TAG, "Unknown characteristic UUID 0x%04x", param->add_char.char_uuid.uuid.uuid16);
            }
            

            break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "Descriptor add Profile%d, status %d, attr_handle %d, service_handle %d",
                app_id, param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
                    
        if(++gl_profile_tab[app_id].chars_init_counter < 2)
        {
            ESP_LOGI(GATTS_TAG, "Adding next characteristic and descriptor...");
            esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[app_id].service_handle, 
                &(gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].uuid),
                gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].perm,
                gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].property,
                &(gl_profile_tab[app_id].chars[gl_profile_tab[app_id].chars_init_counter].attr_val),
                NULL);
            if (add_char_ret) {
                ESP_LOGE(GATTS_TAG, "Failed to add characteristic, error code = %x", add_char_ret);
            }
        } else {
            ESP_LOGI(GATTS_TAG, "All characteristics and descriptors added add_char_descr_evt.");
        }
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service started Profile%d, service_handle %d", app_id, param->start.service_handle);
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic read Profile%d, handle %d", app_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;

        charPtr = get_characteristic_by_handle(&gl_profile_tab[app_id], param->read.handle);
        if(charPtr != NULL)
        {
            rsp.attr_value.len = charPtr->attr_val.attr_len;
            memcpy(rsp.attr_value.value, charPtr->attr_val.attr_value, charPtr->attr_val.attr_len);
        }
        else
        {
            charPtr = get_characteristic_by_descriptor_handle(&gl_profile_tab[app_id], param->read.handle);
            if(charPtr != NULL)
            {
                rsp.attr_value.len = 2; // Length of CCCD value
                memcpy(rsp.attr_value.value, charPtr->cccd_val.attr_value, 2);
            }
            else
            {
                ESP_LOGE(GATTS_TAG, "Unknown handle");
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_ERROR, NULL);
                return;
            }
        }
    
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    case ESP_GATTS_WRITE_EVT: 
        ESP_LOGI(GATTS_TAG, "Write event Profile%d, handle %d", app_id, param->write.handle);

        charPtr = get_characteristic_by_handle(&gl_profile_tab[app_id], param->write.handle);
        if(charPtr != NULL)
        {
            ESP_LOGI(GATTS_TAG, "Characteristic write Profile%d, handle %d", app_id, param->write.handle);
            if (param->write.len > sizeof(charPtr->attr_val.attr_value)) {
                ESP_LOGE(GATTS_TAG, "Write value too long");
                status = ESP_GATT_ERROR;
            } else {
                memcpy(charPtr->attr_val.attr_value, param->write.value, param->write.len);
                charPtr->attr_val.attr_len = param->write.len;
                ESP_LOGI(GATTS_TAG, "Characteristic value: %d, len: %d", charPtr->attr_val.attr_value[0], charPtr->attr_val.attr_len);
                ESP_LOG_BUFFER_HEX(GATTS_TAG, charPtr->attr_val.attr_value, charPtr->attr_val.attr_len);
            }
        }
        else
        {

            charPtr = get_characteristic_by_descriptor_handle(&gl_profile_tab[app_id],param->write.handle);

            if (charPtr != NULL) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                memcpy(charPtr->cccd_val.attr_value, param->write.value,2);
                ESP_LOGI(GATTS_TAG, "Descriptor value: %d", descr_value);
                if (descr_value == 0x0001) {
                    //print as hex in format 0x0001

                    ESP_LOGI(GATTS_TAG, "Notifications enabled for char 0x%04x", charPtr->uuid.uuid.uuid16);
                } else if (descr_value == 0x0002) {
                    ESP_LOGI(GATTS_TAG, "Indications enabled for  char 0x%04x", charPtr->uuid.uuid.uuid16);
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(GATTS_TAG, "Notifications/Indications disabled for char 0x%04x", charPtr->uuid.uuid.uuid16);
                }
            }
            else
            {
                ESP_LOGE(GATTS_TAG, "Unknown descriptor handle %d", param->write.handle);
                status = ESP_GATT_ERROR;
            }
        }
        
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        break;
    
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"Execute write Profile%d", app_id);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
        
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Connected Profile%d, conn_id %d, remote "ESP_BD_ADDR_STR"",
                app_id, param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[app_id].conn_id = param->connect.conn_id;
        if(serviceConnected == false)
        {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x30;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x20;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 600;    // timeout = 400*10ms = 4000ms
            ESP_LOGI(GATTS_TAG, "Connected A, conn_id %u, remote "ESP_BD_ADDR_STR"",
                     param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            
            serviceConnected = true;
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Disconnected Profile%d, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                app_id, ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        if(serviceConnected == true)
        {
            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGI(GATTS_TAG, "Advertising started after disconnection");
        }
        gl_profile_tab[app_id].conn_id = 0xFF;
        gl_profile_tab[app_id].chars_init_counter = 0;
        gl_profile_tab[app_id].service_handle = 0;
        for(int i = 0; i < 2; i++)
        {
            gl_profile_tab[app_id].chars[i].handle = 0;
            gl_profile_tab[app_id].chars[i].descr_handle = 0;            
            // memset(gl_profile_tab[app_id].chars[i].attr_val.attr_value,0, gl_profile_tab[app_id].chars[i].attr_val.attr_len);
            memset(gl_profile_tab[app_id].chars[i].cccd_val.attr_value,0, gl_profile_tab[app_id].chars[i].cccd_val.attr_len);
        }
        gl_profile_tab[app_id].gatts_if = ESP_GATT_IF_NONE;
        
        serviceConnected = false;
        break;
    default:
        break;
    }
}

static void gatts_profile_motor_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    gatts_profile_generic_event_handler(PROFILE_MOTOR_APP_ID, event, gatts_if, param);
}

static void gatts_profile_battery_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    gatts_profile_generic_event_handler(PROFILE_BATTERY_APP_ID, event, gatts_if, param);
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
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
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void ble_driver_srv_task(void)
{
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
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }


    esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name);
    if (set_dev_name_ret){
        ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
    }
#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
    esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (raw_adv_ret){
        ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
    }
    adv_config_done |= adv_config_flag;
    esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
    if (raw_scan_ret){
        ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
    }
    adv_config_done |= scan_rsp_config_flag;
#else
    //config adv data
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret){
        ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
    }
    adv_config_done |= adv_config_flag;
    //config scan response data
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret){
        ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
    }
    adv_config_done |= scan_rsp_config_flag;

#endif

    ret = esp_ble_gatts_app_register(PROFILE_BATTERY_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Battery Service app register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_MOTOR_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Motor Service app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }


    //loop
    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);

    }
    return;
}

void update_battery_level(uint8_t bat_level) {
    battery_level = bat_level;
    if(serviceConnected == true) {
        

        // Update the characteristic value
        esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_LEVEL].handle, sizeof(battery_level), &battery_level);

        // Send notification if enabled
        if (convertArrToUint16(gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_LEVEL].cccd_val.attr_value) == 0x0001) {
            
            ESP_LOGI(GATTS_TAG, "Sending notification for Battery Level: %d%%", battery_level);
            // Send notification to the connected device
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_BATTERY_APP_ID].gatts_if,
                                        gl_profile_tab[PROFILE_BATTERY_APP_ID].conn_id,
                                        gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_LEVEL].handle,
                                        sizeof(battery_level), &battery_level, false);
        }
    }
    else {
        // ESP_LOGI(GATTS_TAG, "Battery Service not connected, skipping update.");
    }

}

void update_battery_voltage(float bat_voltage) {

    battery_voltage = bat_voltage;

    if(serviceConnected == true) {
        uint8_t buf[4];
        convertFloatToUint8Buf(buf,battery_voltage);
        // Update the characteristic value
        esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_VOLTAGE].handle, sizeof(buf), buf);

        // Send notification if enabled
        if (convertArrToUint16(gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_VOLTAGE].cccd_val.attr_value) == 0x0001) {
            
            ESP_LOGI(GATTS_TAG, "Sending notification for Battery Voltage: %.3f", battery_voltage);
            // Send notification to the connected device
            esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_BATTERY_APP_ID].gatts_if,
                                        gl_profile_tab[PROFILE_BATTERY_APP_ID].conn_id,
                                        gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_VOLTAGE].handle,
                                        sizeof(buf), buf, false);
        }
    }
    else {
        // ESP_LOGI(GATTS_TAG, "Battery Service not connected, skipping update.");
    }
}

uint8_t read_motor_speed(void) {
    return motorSpeed;
}

uint8_t read_motor_direction(void) {
    return motorDirection;
}
