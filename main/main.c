/**
 * 
 * Ne pas oublier d'activer le BT en menuconfig +++++
 * 
 * 
*BLE en mode minimaliste basé sur scan/advertise sans se faire chier avec des connexions...
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

#include "driver/uart.h"

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_system.h"

#include "esp_log.h"

#include "driver/gpio.h"

//marche plus début 2019
//#include "controller.h"
//#include "btc_main.h"
#define GPIO_PILOT 4

static const char *MY_TAG = "BLE_PURE";

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

static uint8_t raw_scan_rsp_data[] = {0x0f, 0x09, 0x4c, 0x41, 0x52, 0x4f, 0x51, 0x55, 0x45};

esp_bd_addr_t bdaddr_wl = {0xb8, 0x27, 0xeb, 0x6d, 0x03, 0x70};//B8:27:EB:6D:03:70

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	esp_err_t ret;
	uint8_t *adv_data = NULL;
	
	esp_ble_gap_cb_param_t *p_data = (esp_ble_gap_cb_param_t *) param;
	ESP_LOGI(MY_TAG, "We're in the cb func to the gap module, event = %x", event); //event types: see esp_gap_ble_api.h
	
	//super moche ce if du premier byte de la bdaddr mais le whitelisting ne marche pas...
	if (event == ESP_GAP_BLE_SCAN_RESULT_EVT && param->scan_rst.bda[0] == 0xb8 ) {		

		esp_log_buffer_hex(MY_TAG, param->scan_rst.bda, sizeof(esp_bd_addr_t)); //log bdaddr
		ESP_LOGI(MY_TAG, "SCAN_RESULT_EVT of type %x", p_data->scan_rst.ble_evt_type);
		ESP_LOGI(MY_TAG, "adv_data_len = %i", p_data->scan_rst.adv_data_len);
		ESP_LOGI(MY_TAG, "scan_rsp_len = %i", p_data->scan_rst.scan_rsp_len);
		adv_data = p_data->scan_rst.ble_adv;
			printf("data: ");
			for (int j = 0; j < (p_data->scan_rst.adv_data_len + p_data->scan_rst.scan_rsp_len); j++) {
				printf("%02x ", adv_data[j]);
			}
			printf("\n");
		ESP_LOGI(MY_TAG, "le 10ème byte = %i", adv_data[10]); //conversion en bash: echo $((16#aa))  -->  170
		gpio_pad_select_gpio(GPIO_PILOT);
		gpio_set_direction(GPIO_PILOT, GPIO_MODE_OUTPUT);
		if (adv_data[10] == 170) {
				/**Du code quand la bonne bdaddr et le bon byte à l'endroit où tu l'attends (exple: interrupteur chauffage)**/
				ESP_LOGI(MY_TAG, "Test de l'adv data: trouvé ce que je cherche...");				
				gpio_set_level(GPIO_PILOT, 1);            
			}  else {
				ESP_LOGI(MY_TAG, "Test de l'adv data: pas trouvé ce que je cherche...");
				gpio_set_level(GPIO_PILOT, 0);	
			}
        
        
	}
	
	if (event == ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT) {
		ESP_LOGI(MY_TAG, "it's a SCAN_PARAM_SET_COMPLETE_EVT");		
		ESP_LOGI(MY_TAG, "+++++++status = %i ", p_data->scan_param_cmpl.status); //esp_bt_status_t
		
		ret = esp_ble_gap_start_scanning(10); //duration in s
	}
	
	if (event == ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT) {
		ESP_LOGI(MY_TAG, "it's a ADV_DATA_RAW_SET_COMPLETE_EVT");
			
		ret = esp_ble_gap_start_advertising(&adv_params); //duration in s
	}
	
}

void app_main()
{
    esp_err_t status;

	// Initialize NVS flash storage with layout given in the partition table
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) 
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ESP_LOGI(MY_TAG, "Enabling Bluetooth Controller");
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();    
    
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(MY_TAG, "%s init controller failed\n", __func__);
        return;
    }

	//Peut poser pb:
	//2018: "Now only support BTDM" (components/bt/include/bt.h) donc je tentais BLE ça plantait
	//2019: ya que ESP_BT_MODE_BLE qui passe, va comprendre....
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(MY_TAG, "%s Enable controller failed\n", __func__);
        return;
    }

    ESP_LOGI(MY_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(MY_TAG, "%s init bluetooth failed", __func__);
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(MY_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }
    
    //register the scan callback function to the gap module
    if ((ret = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(MY_TAG, "gap register error, error code = %x", ret);
        return;
    }
    
    if ((ret = esp_ble_gap_update_whitelist(true, bdaddr_wl)) != ESP_OK) {
        ESP_LOGE(MY_TAG, "gap update whitelist error, error code = %x", ret);
        return;
    }
    
    
    
    //lance le scan dans un loop, car dans les params du scan tu dis combien de temps il scanne, et c'est pas infini
    while( true ) {
	    ret = esp_ble_gap_set_scan_params(&ble_scan_params);
		ESP_LOGI(MY_TAG, "******set scan param returns = %i", ret); //will generate ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT
		vTaskDelay(15000/portTICK_PERIOD_MS); //millisecondes
	}
    
    /**Si tu veux advertiser / émettre une scan rsp**/
	//ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, 9); //uint8_t *raw_data, uint32_t raw_data_len, will generate ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT
    //ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, 13); //uint8_t *raw_data, uint32_t raw_data_len, will generate ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT
 
 
 
}

