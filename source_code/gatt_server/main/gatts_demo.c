#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#define TAG "BLE_WIFI"
#define SERVICE_UUID     0x00FF
#define CHAR_UUID_SSID   0xFF01
#define CHAR_UUID_PASS   0xFF02

static uint16_t ssid_handle, pass_handle;
static uint8_t ssid[32] = {0}, password[64] = {0};
static bool ssid_ok = false, pass_ok = false;
static uint16_t service_handle;

static uint8_t service_uuid[2] = {0x00, 0xFF};  // ✅ ĐÚNG THỨ TỰ BYTE CHUẨN UUID 0x00FF

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void try_connect_wifi() {
    wifi_config_t wifi_cfg = {0};
    strcpy((char*)wifi_cfg.sta.ssid, (char*)ssid);
    strcpy((char*)wifi_cfg.sta.password, (char*)password);

    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    esp_wifi_connect();
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&adv_params);
    }
}

static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "BLE client connected. Conn ID: %d, Address: "ESP_BD_ADDR_STR,
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGW(TAG, "BLE client disconnected. Reason: 0x%02x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_REG_EVT:
        esp_ble_gap_set_device_name("ESP32_PROV");
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gatts_create_service(gatts_if, &(esp_gatt_srvc_id_t){
            .is_primary = true,
            .id.inst_id = 0,
            .id.uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = SERVICE_UUID}}
        }, 6);
        break;

    case ESP_GATTS_CREATE_EVT:
        service_handle = param->create.service_handle;
        ESP_LOGI(TAG, "SERVICE CREATED, handle = %d", service_handle);
        esp_ble_gatts_start_service(service_handle);
        esp_ble_gatts_add_char(service_handle, &(esp_bt_uuid_t){
            .len = ESP_UUID_LEN_16, .uuid.uuid16 = CHAR_UUID_SSID
        }, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.uuid.uuid16 == CHAR_UUID_SSID) {
            ssid_handle = param->add_char.attr_handle;
            esp_ble_gatts_add_char(service_handle, &(esp_bt_uuid_t){
                .len = ESP_UUID_LEN_16, .uuid.uuid16 = CHAR_UUID_PASS
            }, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_UUID_PASS) {
            pass_handle = param->add_char.attr_handle;
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == ssid_handle) {
            memset(ssid, 0, sizeof(ssid));
            memcpy(ssid, param->write.value, param->write.len);
            ssid_ok = true;
            ESP_LOGI(TAG, "Received SSID: %s", ssid);
        } else if (param->write.handle == pass_handle) {
            memset(password, 0, sizeof(password));
            memcpy(password, param->write.value, param->write.len);
            pass_ok = true;
            ESP_LOGI(TAG, "Received PASS: %s", password);
        }

        if (ssid_ok && pass_ok) {
            ssid_ok = pass_ok = false;
            ESP_LOGI(TAG, "\n==== Received Both SSID & PASSWORD ====");
            ESP_LOGI(TAG, "➡️ SSID: %s", ssid);
            ESP_LOGI(TAG, "➡️ PASSWORD: %s", password);
            try_connect_wifi();
        }
        break;

    default:
        break;
    }
}

void app_main(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gatts_register_callback(gatts_cb);
    esp_ble_gap_register_callback(gap_cb);
    esp_ble_gatts_app_register(0);
}
