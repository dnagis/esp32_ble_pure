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
#include "esp_bt_main.h"
#include "esp_system.h"
#include "btc_main.h"


static const char *MON_TAG = "BLE_PURE";

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	ESP_LOGI(MON_TAG, "On est dans la callback function to the gap module");
	
}


void ble_client_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(MON_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(MON_TAG, "gap register error, error code = %x", status);
        return;
    }
    
}



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

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM); //Now only support BTDM (components/bt/include/bt.h)
    if (ret) {
        ESP_LOGE(MON_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ESP_LOGI(MON_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(MON_TAG, "%s init bluetooth failed", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(MON_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }
    
    ble_client_appRegister();
    ret = esp_ble_gap_start_scanning(60); //miracle, quand je mets ça ça passe enfin dans la callback: je reçois des packets!!! YEEEESSSSSSS
 
}

