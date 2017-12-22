/**
*BLE en mode minimaliste pour comprendre ce que je fais...
* 
* examples/ble_adv me permet de faire de l'advertise en commandes HCI très 'raw'
* le problème c'est que si je veux récupérer de l'info dans l'esp32 il faut que je scanne un peu pour avoir de la scan rsp
* et là, faut que j'écoute... Et je vois pas comment faire pour écouter en hci raw. Du coup: 
* 
* j'utilise les defs dans: bluedroid/api/include/esp_gap_ble_api.h
* 
* je me sers de spp_client_demo.c comme point de départ.
* 
* 
 * branch git: esp_gap_ble --> git push -u origin HEAD pour push chez github.
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
	ESP_LOGI(MON_TAG, "On est dans la callback function to the gap module, event = %i", event);
	
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

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM); //"Now only support BTDM" (components/bt/include/bt.h) donc si tu tentes BLE ça plante
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
    
    //register the scan callback function to the gap module
    if ((ret = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(MON_TAG, "gap register error, error code = %x", ret);
        return;
    }
    
    ret = esp_ble_gap_start_scanning(10); //lance le scan, la callback va être appelée quand de l'event arrive. Arg=duration en secondes
 
}

