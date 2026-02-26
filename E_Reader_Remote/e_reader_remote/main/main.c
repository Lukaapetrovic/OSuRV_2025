/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_log.h"
#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include "driver/gpio.h"
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define REPORT_PROTOCOL_KEYBOARD_REPORT_SIZE   (8)
#define REPORT_BUFFER_SIZE                     REPORT_PROTOCOL_KEYBOARD_REPORT_SIZE

// GPIO pin definitions for e-reader remote
#define GPIO_LEFT_ARROW     19
#define GPIO_RIGHT_ARROW    21
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_LEFT_ARROW) | (1ULL<<GPIO_RIGHT_ARROW))

// HID keyboard key codes
#define HID_KEY_LEFT_ARROW   0x50
#define HID_KEY_RIGHT_ARROW  0x4F

static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;

typedef struct {
    esp_hidd_app_param_t app_param;
    esp_hidd_qos_param_t both_qos;
    uint8_t protocol_mode;
    SemaphoreHandle_t keyboard_mutex;
    TaskHandle_t keyboard_task_hdl;
    uint8_t buffer[REPORT_BUFFER_SIZE];
} local_param_t;

static local_param_t s_local_param = {0};

// HID report descriptor for a keyboard
uint8_t hid_keyboard_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (Num Lock)
    0x29, 0x05,        //   Usage Maximum (Kana)
    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x03,        //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x29, 0x65,        //   Usage Maximum (0x65)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};

// GPIO setup function
void setup_gpio(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;  // Enable internal pull-ups
    gpio_config(&io_conf);
    
    ESP_LOGI("GPIO", "GPIO configured - Left: %d, Right: %d", GPIO_LEFT_ARROW, GPIO_RIGHT_ARROW);
}

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

const int hid_keyboard_descriptor_len = sizeof(hid_keyboard_descriptor);

/**
 * @brief Integrity check of the report ID and report type for GET_REPORT request from HID host.
 */
bool check_report_id_type(uint8_t report_id, uint8_t report_type)
{
    bool ret = false;
    xSemaphoreTake(s_local_param.keyboard_mutex, portMAX_DELAY);
    do {
        if (report_type != ESP_HIDD_REPORT_TYPE_INPUT) {
            break;
        }
        if (s_local_param.protocol_mode == ESP_HIDD_BOOT_MODE) {
            if (report_id == ESP_HIDD_BOOT_REPORT_ID_KEYBOARD) {
                ret = true;
                break;
            }
        } else {
            if (report_id == 1) {  // Report ID for keyboard
                ret = true;
                break;
            }
        }
    } while (0);

    if (!ret) {
        esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
    }
    xSemaphoreGive(s_local_param.keyboard_mutex);
    return ret;
}

// Send keyboard report
void send_keyboard_report(uint8_t modifier, uint8_t keycode)
{
    uint8_t report_id;
    uint16_t report_size;
    xSemaphoreTake(s_local_param.keyboard_mutex, portMAX_DELAY);
    
    if (s_local_param.protocol_mode == ESP_HIDD_REPORT_MODE) {
        report_id = 1;  // Report ID for keyboard
        report_size = REPORT_PROTOCOL_KEYBOARD_REPORT_SIZE;
        s_local_param.buffer[0] = modifier;   // Modifier keys
        s_local_param.buffer[1] = 0;          // Reserved
        s_local_param.buffer[2] = keycode;    // Key code
        s_local_param.buffer[3] = 0;          // Additional keys
        s_local_param.buffer[4] = 0;
        s_local_param.buffer[5] = 0;
        s_local_param.buffer[6] = 0;
        s_local_param.buffer[7] = 0;
    } else {
        // Boot Mode
        report_id = ESP_HIDD_BOOT_REPORT_ID_KEYBOARD;
        report_size = ESP_HIDD_BOOT_REPORT_SIZE_KEYBOARD - 1;
        s_local_param.buffer[0] = modifier;   // Modifier keys
        s_local_param.buffer[1] = 0;          // Reserved
        s_local_param.buffer[2] = keycode;    // Key code
        s_local_param.buffer[3] = 0;          // Additional keys
        s_local_param.buffer[4] = 0;
        s_local_param.buffer[5] = 0;
        s_local_param.buffer[6] = 0;
        s_local_param.buffer[7] = 0;
    }
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, report_id, report_size, s_local_param.buffer);
    xSemaphoreGive(s_local_param.keyboard_mutex);
}

// Send a single key press (press + release)
void send_key(uint8_t keycode, const char* key_name)
{
    ESP_LOGI("KEYBOARD", "Sending %s key", key_name);
    
    // Send key press
    send_keyboard_report(0, keycode);
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Hold for 50ms
    
    // Send key release
    send_keyboard_report(0, 0);
}

// GPIO-based keyboard task for e-reader remote
void keyboard_task(void *pvParameters)
{
    const char *TAG = "keyboard_task";

    ESP_LOGI(TAG, "E-Reader Remote Keyboard Task Starting");
    ESP_LOGI(TAG, "GPIO %d = Left Arrow (Previous Page)", GPIO_LEFT_ARROW);
    ESP_LOGI(TAG, "GPIO %d = Right Arrow (Next Page)", GPIO_RIGHT_ARROW);
    
    static bool left_last_reading = true;    // Last GPIO reading
    static bool right_last_reading = true;
    static bool left_stable_state = true;   // Last confirmed stable state
    static bool right_stable_state = true;
    static TickType_t left_last_change = 0;
    static TickType_t right_last_change = 0;
    
    const TickType_t debounce_delay = 50 / portTICK_PERIOD_MS;  // Back to 50ms
    
    // Log initial GPIO states
    ESP_LOGI(TAG, "Initial GPIO states - Left: %d, Right: %d", 
             gpio_get_level(GPIO_LEFT_ARROW), gpio_get_level(GPIO_RIGHT_ARROW));
    
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        
        // Read current GPIO states
        bool left_current = gpio_get_level(GPIO_LEFT_ARROW);
        bool right_current = gpio_get_level(GPIO_RIGHT_ARROW);
        
        // LEFT BUTTON LOGIC
        if (left_current != left_last_reading) {
            // Reading changed, reset the debounce timer
            left_last_change = now;
            ESP_LOGI(TAG, "Left GPIO changed: %d -> %d", left_last_reading, left_current);
        }
        left_last_reading = left_current;
        
        // Check if enough time has passed for a stable reading
        if ((now - left_last_change) > debounce_delay) {
            // Reading has been stable for debounce period
            if (left_current != left_stable_state) {
                // State change confirmed
                if (left_stable_state == true && left_current == false) {
                    // Button pressed (1 -> 0)
                    ESP_LOGI(TAG, "LEFT BUTTON PRESSED!");
                    send_key(HID_KEY_LEFT_ARROW, "LEFT ARROW");
                } else if (left_stable_state == false && left_current == true) {
                    // Button released (0 -> 1)
                    ESP_LOGI(TAG, "LEFT BUTTON RELEASED!");
                }
                left_stable_state = left_current;
            }
        }
        
        // RIGHT BUTTON LOGIC (same pattern)
        if (right_current != right_last_reading) {
            // Reading changed, reset the debounce timer
            right_last_change = now;
            ESP_LOGI(TAG, "Right GPIO changed: %d -> %d", right_last_reading, right_current);
        }
        right_last_reading = right_current;
        
        // Check if enough time has passed for a stable reading
        if ((now - right_last_change) > debounce_delay) {
            // Reading has been stable for debounce period
            if (right_current != right_stable_state) {
                // State change confirmed
                if (right_stable_state == true && right_current == false) {
                    // Button pressed (1 -> 0)
                    ESP_LOGI(TAG, "RIGHT BUTTON PRESSED!");
                    send_key(HID_KEY_RIGHT_ARROW, "RIGHT ARROW");
                } else if (right_stable_state == false && right_current == true) {
                    // Button released (0 -> 1)
                    ESP_LOGI(TAG, "RIGHT BUTTON RELEASED!");
                }
                right_stable_state = right_current;
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "esp_bt_gap_cb";
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %06"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%06"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;
    default:
        ESP_LOGI(TAG, "event: %d", event);
        break;
    }
    return;
}

void bt_app_task_start_up(void)
{
    s_local_param.keyboard_mutex = xSemaphoreCreateMutex();
    memset(s_local_param.buffer, 0, REPORT_BUFFER_SIZE);
    xTaskCreate(keyboard_task, "keyboard_task", 4 * 1024, NULL, configMAX_PRIORITIES - 3, &s_local_param.keyboard_task_hdl);
    return;
}

void bt_app_task_shut_down(void)
{
    if (s_local_param.keyboard_task_hdl) {
        vTaskDelete(s_local_param.keyboard_task_hdl);
        s_local_param.keyboard_task_hdl = NULL;
    }

    if (s_local_param.keyboard_mutex) {
        vSemaphoreDelete(s_local_param.keyboard_mutex);
        s_local_param.keyboard_mutex = NULL;
    }
    return;
}

void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    static const char *TAG = "esp_bt_hidd_cb";
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        if (param->init.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters");
            esp_bt_hid_device_register_app(&s_local_param.app_param, &s_local_param.both_qos, &s_local_param.both_qos);
        } else {
            ESP_LOGE(TAG, "init hidd failed!");
        }
        break;
    case ESP_HIDD_DEINIT_EVT:
        break;
    case ESP_HIDD_REGISTER_APP_EVT:
        if (param->register_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters success!");
            ESP_LOGI(TAG, "setting to connectable, discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            if (param->register_app.in_use) {
                ESP_LOGI(TAG, "start virtual cable plug!");
                esp_bt_hid_device_connect(param->register_app.bd_addr);
            }
        } else {
            ESP_LOGE(TAG, "setting hid parameters failed!");
        }
        break;
    case ESP_HIDD_UNREGISTER_APP_EVT:
        if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "unregister app success!");
        } else {
            ESP_LOGE(TAG, "unregister app failed!");
        }
        break;
    case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS) {
            if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                ESP_LOGI(TAG, "connecting...");
            } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
                ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                         param->open.bd_addr[5]);
                bt_app_task_start_up();
                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "open failed!");
        }
        break;
    case ESP_HIDD_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_CLOSE_EVT");
        if (param->close.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                ESP_LOGI(TAG, "disconnecting...");
            } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    case ESP_HIDD_SEND_REPORT_EVT:
        if (param->send_report.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d", param->send_report.report_id,
                     param->send_report.report_type);
        } else {
            ESP_LOGE(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d, status:%d, reason:%d",
                     param->send_report.report_id, param->send_report.report_type, param->send_report.status,
                     param->send_report.reason);
        }
        break;
    case ESP_HIDD_REPORT_ERR_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
        break;
    case ESP_HIDD_GET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                 param->get_report.report_type, param->get_report.buffer_size);
        if (check_report_id_type(param->get_report.report_id, param->get_report.report_type)) {
            uint8_t report_id;
            uint16_t report_len;
            if (s_local_param.protocol_mode == ESP_HIDD_REPORT_MODE) {
                report_id = 1;  // Keyboard report ID
                report_len = REPORT_PROTOCOL_KEYBOARD_REPORT_SIZE;
            } else {
                // Boot Mode
                report_id = ESP_HIDD_BOOT_REPORT_ID_KEYBOARD;
                report_len = ESP_HIDD_BOOT_REPORT_SIZE_KEYBOARD - 1;
            }
            xSemaphoreTake(s_local_param.keyboard_mutex, portMAX_DELAY);
            esp_bt_hid_device_send_report(param->get_report.report_type, report_id, report_len, s_local_param.buffer);
            xSemaphoreGive(s_local_param.keyboard_mutex);
        } else {
            ESP_LOGE(TAG, "check_report_id failed!");
        }
        break;
    case ESP_HIDD_SET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT");
        break;
    case ESP_HIDD_SET_PROTOCOL_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT");
        if (param->set_protocol.protocol_mode == ESP_HIDD_BOOT_MODE) {
            ESP_LOGI(TAG, "  - boot protocol");
        } else if (param->set_protocol.protocol_mode == ESP_HIDD_REPORT_MODE) {
            ESP_LOGI(TAG, "  - report protocol");
        }
        xSemaphoreTake(s_local_param.keyboard_mutex, portMAX_DELAY);
        s_local_param.protocol_mode = param->set_protocol.protocol_mode;
        xSemaphoreGive(s_local_param.keyboard_mutex);
        break;
    case ESP_HIDD_INTR_DATA_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
        break;
    case ESP_HIDD_VC_UNPLUG_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
        if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    default:
        break;
    }
}

void app_main(void)
{
    const char *TAG = "app_main";
    esp_err_t ret;
    char bda_str[18] = {0};

    ESP_LOGI(TAG, "Starting E-Reader Remote (Classic BT Keyboard)");

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Setup GPIO for e-reader remote buttons
    setup_gpio();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "enable bluedroid failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "setting device name");
    esp_bt_gap_set_device_name(local_device_name);

    ESP_LOGI(TAG, "setting cod major, peripheral keyboard");
    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_KEYBOARD;  // Keyboard device
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Initialize HID SDP information and L2CAP parameters.
    do {
        s_local_param.app_param.name = "E-Reader Remote";
        s_local_param.app_param.description = "ESP32 E-Reader Remote Control";
        s_local_param.app_param.provider = "ESP32";
        s_local_param.app_param.subclass = ESP_HID_CLASS_KBD; // Keyboard class
        s_local_param.app_param.desc_list = hid_keyboard_descriptor;
        s_local_param.app_param.desc_list_len = hid_keyboard_descriptor_len;

        memset(&s_local_param.both_qos, 0, sizeof(esp_hidd_qos_param_t));
    } while (0);

    // Report Protocol Mode is the default mode
    s_local_param.protocol_mode = ESP_HIDD_REPORT_MODE;

    ESP_LOGI(TAG, "register hid device callback");
    esp_bt_hid_device_register_callback(esp_bt_hidd_cb);

    ESP_LOGI(TAG, "starting hid keyboard device");
    esp_bt_hid_device_init();

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
    ESP_LOGI(TAG, "E-Reader Remote ready!");
    ESP_LOGI(TAG, "GPIO %d = Left Arrow (Previous Page)", GPIO_LEFT_ARROW);
    ESP_LOGI(TAG, "GPIO %d = Right Arrow (Next Page)", GPIO_RIGHT_ARROW);
    ESP_LOGI(TAG, "Connect buttons: one side to GPIO, other side to GND");
}