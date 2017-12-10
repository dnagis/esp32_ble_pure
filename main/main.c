// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * Inspiré fortement de esp-idf/examples/bluetooth/ble_adv
 * 
 * https://git-scm.com/book/fr/v1/Les-branches-avec-Git-Brancher-et-fusionner%C2%A0:-les-bases
 * 
 * https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/02/10/bluetooth_advertisin-hGsf
 * 
 * https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
 * 
 * hcitool lescan
	LE Scan ...
	30:AE:A4:04:C3:5A ESP-BLE-HELLO
	30:AE:A4:04:C3:5A (unknown)
 * 
 * 
 * btmon:
 * > HCI Event: LE Meta Event (0x3e) plen 12                                                                      #98 [hci0] 97.848436
      LE Advertising Report (0x02)
        Num reports: 1
        Event type: Scan response - SCAN_RSP (0x04)
        Address type: Public (0x00)
        Address: 30:AE:A4:04:C3:5A (OUI 30-AE-A4)
        Data length: 0
        RSSI: -32 dBm (0xe0)
@ Device Found: 30:AE:A4:04:C3:5A (1) rssi -32 flags 0x0000
        02 01 06 0e 09 45 53 50 2d 42 4c 45 2d 48 45 4c  .....ESP-BLE-HEL
        4c 4f                                            LO 
 * 
 * 
 * 
 * 
 */


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bt.h"
#include "esp_log.h"
#include "nvs_flash.h"


static const char *MON_TAG = "BLE_PURE";


#define HCI_H4_CMD_PREAMBLE_SIZE           (4)

/*  HCI Command opcode group field(OGF) */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_BLE_WRITE_ADV_ENABLE           (0x000A | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_PARAMS           (0x0006 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)

#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)

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
    //... et puis l'histoire de chech_send_available: vu qu'il est toujours available: fuck it...
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
	hci_cmd_send_ble_adv_start();
	vTaskDelay(60000 / portTICK_PERIOD_MS); //si je fais pas ça le bestiau redémarre tout le temps.... chiant
    

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

