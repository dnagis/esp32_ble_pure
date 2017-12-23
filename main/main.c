/**
*BLE en mode minimaliste pour comprendre ce que je fais...
* 
* examples/ble_adv me permet de faire de l'advertise en commandes HCI très 'raw'
* le problème c'est que si je veux récupérer de l'info dans l'esp32 il faut que je scanne un peu pour avoir de la scan rsp
* et là, faut que j'écoute... Et je vois pas comment faire pour écouter en hci raw. Du coup: 
* 
* j'utilise les defs dans: components/bt/bluedroid/api/include/esp_gap_ble_api.h
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

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ONLY_WLST,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

static uint8_t raw_adv_data[] = {0x02, 0x01, 0x06, 0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd, 0xab, 0xcd, 0xee};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	esp_err_t ret;
	uint8_t *adv_data = NULL;
	
	esp_ble_gap_cb_param_t *p_data = (esp_ble_gap_cb_param_t *) param;
	ESP_LOGI(MON_TAG, "On est dans la callback function to the gap module, event = %x", event); //type de event: esp_gap_ble_api.h
	
	if (event == ESP_GAP_BLE_SCAN_RESULT_EVT) {		
		/*récupérer data: defs de esp_ble_gap_cb_param_t dans esp_gap_ble_api.h. Quand scan result -> param->scan_rst.bda (esp_bd_addr_t)*/
		esp_log_buffer_hex(MON_TAG, param->scan_rst.bda, sizeof(esp_bd_addr_t)); //adresse
		ESP_LOGI(MON_TAG, "c'est du SCAN_RESULT_EVT de type %i", p_data->scan_rst.ble_evt_type);
		ESP_LOGI(MON_TAG, "adv_data_len = %i", p_data->scan_rst.adv_data_len);
		//ESP_LOGI(MON_TAG, "c'est du SCAN_RESULT_EVT de type %x", p_data->scan_rst.ble_evt_type);
		adv_data = p_data->scan_rst.ble_adv;
            printf("data: ");
            for (int j = 0; j < p_data->scan_rst.adv_data_len; j++) {
                printf("%02x ", adv_data[j]);
            }
            printf("\n");
	}
	
	if (event == ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT) {
		ESP_LOGI(MON_TAG, "c'est du SCAN_PARAM_SET_COMPLETE_EVT");
		
		ESP_LOGI(MON_TAG, "+++++++status = %i ", p_data->scan_param_cmpl.status); //esp_bt_status_t
		
		ret = esp_ble_gap_start_scanning(30); //duration en s
	}
	
	if (event == ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT) {
		ESP_LOGI(MON_TAG, "c'est du ADV_DATA_RAW_SET_COMPLETE_EVT");
			
		ret = esp_ble_gap_start_advertising(&adv_params); //duration en s
	}
	
}

void app_main()
{
    esp_err_t ret;
    uint16_t taille_wl;
    esp_bd_addr_t bda_wl = {0x81,0x82,0x83,0x84,0x85,0x86};
    
    
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
    
    
    ret = esp_ble_gap_get_whitelist_size(&taille_wl);
		ESP_LOGI(MON_TAG, "******retour de get wl sz AVANT update wl = %i avec ret = %i", taille_wl, ret);
	ret = esp_ble_gap_update_whitelist(1, bda_wl); //bool add_remove, esp_bd_addr_t remote_bda
		ESP_LOGI(MON_TAG, "******retour de update wl ret = %i", ret);
		
	//vTaskDelay(3000 / portTICK_PERIOD_MS); //en millisecondes
	
	ret = esp_ble_gap_get_whitelist_size(&taille_wl);
		ESP_LOGI(MON_TAG, "******retour de get wl sz APRES update wl = %i avec ret = %i", taille_wl, ret);
    ret = esp_ble_gap_set_scan_params(&ble_scan_params);
		ESP_LOGI(MON_TAG, "******retour de set scan param = %i", ret); //ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT généré
 
    

    ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, 13); //uint8_t *raw_data, uint32_t raw_data_len, va générer ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT
 
 
}

