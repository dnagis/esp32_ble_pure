/**
 * Basé sur esp-idf/examples/bluetooth/ble_adv
 * 
 * * 
 * Commandes HCI dans core specs: " Host Controller Interface Functional Specification Vol 2, Part E "
 * 		LE Set Advertising Parameters Command p 1251
 * 		LE Set Advertising Data Command p 1256
 * 		LE Set Scan Response Data Command p 1257
 * 
 * 
 * 
 * --> Scan, début le 201217 
 * LE Set Scan Parameters Command. Core Specs p 1261. Vol. 2 Part E. HCI Func Specs
 * 
 */


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bt.h"
#include "esp_log.h"
#include "nvs_flash.h"

//temperature
#include <stdlib.h>
#include <math.h>

float temperature;
static const char *MON_TAG = "BLE_PURE";


#define HCI_H4_CMD_PREAMBLE_SIZE           (4)

/*  HCI Command opcode group field(OGF) */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_BLE_WRITE_ADV_ENABLE           (0x000A | HCI_GRP_BLE_CMDS)  //core specs p 1259
#define HCI_BLE_WRITE_ADV_PARAMS           (0x0006 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_SCAN_RESP_DATA       (0x0009 | HCI_GRP_BLE_CMDS) //core specs HCI func specs p 1257

#define HCI_BLE_WRITE_SCAN_PARAMS           (0x000B | HCI_GRP_BLE_CMDS) //core specs p 1261
#define HCI_BLE_WRITE_SCAN_ENABLE           (0x000C | HCI_GRP_BLE_CMDS) //core specs p 1264

#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)

#define HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAMS    (7) //somme des octets des command params
#define HCIC_PARAM_SIZE_WRITE_SCAN_ENABLE        (2)

#define BD_ADDR_LEN     (6)                     /* Device address length */
typedef uint8_t bd_addr_t[BD_ADDR_LEN];         /* Device address */

#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};

static uint8_t hci_cmd_buf[128];

/*
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
static void controller_rcv_pkt_ready(void)
{
    printf("controller rcv pkt ready\n");
}

/*
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    printf("host rcv pkt: ");
    for (uint16_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

static uint16_t make_cmd_reset(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);
    return HCI_H4_CMD_PREAMBLE_SIZE;
}

static uint16_t make_cmd_ble_set_adv_enable (uint8_t *buf, uint8_t adv_enable)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM (buf, adv_enable);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_ADV_ENABLE;
}

static uint16_t make_cmd_ble_set_adv_param (uint8_t *buf, uint16_t adv_int_min, uint16_t adv_int_max,
        uint8_t adv_type, uint8_t addr_type_own,
        uint8_t addr_type_dir, bd_addr_t direct_bda,
        uint8_t channel_map, uint8_t adv_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS );

    UINT16_TO_STREAM (buf, adv_int_min);
    UINT16_TO_STREAM (buf, adv_int_max);
    UINT8_TO_STREAM (buf, adv_type);
    UINT8_TO_STREAM (buf, addr_type_own);
    UINT8_TO_STREAM (buf, addr_type_dir);
    BDADDR_TO_STREAM (buf, direct_bda);
    UINT8_TO_STREAM (buf, channel_map);
    UINT8_TO_STREAM (buf, adv_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS;
}


static uint16_t make_cmd_ble_set_adv_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}

static void hci_cmd_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_start(void)
{
    uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void)
{
    uint16_t adv_intv_min = 256; // 160ms
    uint16_t adv_intv_max = 256; // 160ms
    uint8_t adv_type = 0; // connectable undirected advertising (ADV_IND)
    uint8_t own_addr_type = 0; // Public Device Address
    uint8_t peer_addr_type = 0; // Public Device Address
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t adv_chn_map = 0x07; // 37, 38, 39
    uint8_t adv_filter_policy = 0; // Process All Conn and Scan

    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                  adv_intv_min,
                  adv_intv_max,
                  adv_type,
                  own_addr_type,
                  peer_addr_type,
                  peer_addr,
                  adv_chn_map,
                  adv_filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_data(void)
{
    char *adv_name = "ESP-BLE-HELLO3";
    uint8_t name_len = (uint8_t)strlen(adv_name);
    uint8_t adv_data[31] = {0x02, 0x01, 0x06, 0x0, 0x09}; 	//0x01: «Flags» 0x09: «Complete Local Name» bluetooth.com/specifications/assigned-numbers/generic-access-profile
    uint8_t adv_data_len;
    
    /**02 01 06 0e 09 45 53 50 2d 42 4c 45 2d 48 45 4c  .....ESP-BLE-HELLO
        4c 4f */

    adv_data[3] = name_len + 1;
    for (int i = 0; i < name_len; i++) {
        adv_data[5 + i] = (uint8_t)adv_name[i];
    }
    adv_data_len = 5 + name_len;
	//core specs p 1256
    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

/***SET SCAN RESP... VVNX***/
static uint16_t make_cmd_ble_set_scan_resp_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_SCAN_RESP_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}

static void hci_cmd_send_ble_set_scan_resp_data(void)
{	
	temperature = 20.5; //set la temp
	double temp_abs = fabs(temperature);
	int intPart = (int) temp_abs; //partie avant la virgule
	float fracPart = temp_abs - intPart;
	int decPart = (int)((fracPart * 10)+0.5); //partie après la virgule
	
	uint8_t adv_data[31];
	adv_data[0] = intPart;
	adv_data[1] = decPart;
	adv_data[2] = (temperature < 0) ? 0 : 1 ; //"sign" 0 si <0 sinon 1
	
	//uint8_t adv_data[31] = {0x04, 0x05, 0x06, 0x04, 0x08, 0x09, 0x10};
	uint8_t adv_data_len = 3;
	uint16_t sz = make_cmd_ble_set_scan_resp_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}
/***SET SCAN RESP... VVNX***/

/***SET SCAN PARAMS... VVNX***/
static uint16_t make_cmd_ble_set_scan_param (uint8_t *buf, uint8_t scan_type, uint16_t scan_intv, uint16_t scan_wndw, 
	uint8_t own_addr_type, uint8_t scan_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_SCAN_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAMS );

	UINT8_TO_STREAM (buf, scan_type);
	UINT16_TO_STREAM (buf, scan_intv);
	UINT16_TO_STREAM (buf, scan_wndw);
	UINT8_TO_STREAM (buf, own_addr_type);
	UINT8_TO_STREAM (buf, scan_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAMS;
}

static void hci_cmd_send_ble_set_scan_param(void)
{
    uint8_t scan_type = 0;
    uint16_t scan_intv = 10; //pas bien compris la conversion....
    uint16_t scan_wndw = 10; //pas bien compris la conversion....
    uint8_t own_addr_type = 0;
    uint8_t scan_filter_policy = 0;
    
   uint16_t sz = make_cmd_ble_set_scan_param(hci_cmd_buf,
                  scan_type,
                  scan_intv,
                  scan_wndw,
                  own_addr_type,
                  scan_filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}
/***SET SCAN PARAMS... VVNX***/

/***SET SCAN ENABLE... VVNX***/
static uint16_t make_cmd_ble_set_scan_enable (uint8_t *buf, uint8_t scan_enable, uint8_t filter_duplicates)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_SCAN_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_SCAN_ENABLE);
    UINT8_TO_STREAM (buf, scan_enable);
    UINT8_TO_STREAM (buf, filter_duplicates);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_SCAN_ENABLE;
}

static void hci_cmd_send_ble_scan_enable(void)
{
	uint8_t scan_enable = 1;
	uint8_t filter_duplicates = 0;
    uint16_t sz = make_cmd_ble_set_scan_enable (hci_cmd_buf, scan_enable, filter_duplicates);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}
/***SET SCAN ENABLE... VVNX***/
/*
 * @brief: send HCI commands to perform BLE advertising;
 */
void bleAdvtTask(void *pvParameters)
{
    int cmd_cnt = 0;
    bool send_avail = false;
    esp_vhci_host_register_callback(&vhci_host_cb);
    printf("BLE advt task start\n");
    //Perso je vois pas l'interêt de la boucle... L'advertisement se fait en continu même si tu fais pas de boucle...
    //... et puis l'histoire de check_send_available: vu qu'il est toujours available: fuck it...
    /*while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        send_avail = esp_vhci_host_check_send_available();
        if (send_avail) {
            switch (cmd_cnt) {
            case 0: hci_cmd_send_reset(); ++cmd_cnt; break;
            case 1: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;
            case 2: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; break;
            case 3: hci_cmd_send_ble_adv_start(); ++cmd_cnt; break;
            }
        }
        printf("BLE Advertise, flag_send_avail: %d, cmd_sent: %d\n", send_avail, cmd_cnt);
    }*/
	hci_cmd_send_reset();
	hci_cmd_send_ble_set_adv_param();
	hci_cmd_send_ble_set_adv_data();
	hci_cmd_send_ble_set_scan_resp_data();
	hci_cmd_send_ble_set_scan_param();
	hci_cmd_send_ble_adv_start();
	vTaskDelay(600000 / portTICK_PERIOD_MS); //si je fais pas ça le bestiau redémarre tout le temps.... chiant
}


void app_main()
{
    // Initialize NVS. "Non-volatile storage (NVS) library is designed to store key-value pairs in flash."
    esp_err_t ret = nvs_flash_init();    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );    

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT(); //components/bt/include/bt.h
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(MON_TAG, "%s initialize controller failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM); //BTDM = "dual mode. Si je ne veux que LE -> ERROR enable controller failed, error code = 102
    if (ret) {
        ESP_LOGE(MON_TAG, "%s enable controller failed, error code = %x\n", __func__, ret);
        return;
    }

	xTaskCreatePinnedToCore(&bleAdvtTask, "bleAdvtTask", 2048, NULL, 5, NULL, 0);

	ESP_LOGI(MON_TAG, "Vincent, fin de app_main()...");
}

