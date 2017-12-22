/**
*
* J'arrive pas à récupérer les packets en lançant le scan direct en commandes HCI donc je passe
* à la version: bluedroid/api/include/esp_gap_ble_api.h
* 
* je me sers de spp_client_demo.c
* 
* 
 * 
 */


#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "controller.h"
#include "driver/uart.h"

#include "bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "btc_main.h"
#include "esp_gatt_common_api.h"


static const char *MON_TAG = "BLE_PURE";



void app_main()
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    nvs_flash_init();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(MON_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(MON_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ESP_LOGI(MON_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(MON_TAG, "%s init bluetooth failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(MON_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }
}

