// STD includes
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

// for delaying MS
#include "esp_task_wdt.h"

// GPIO handle
#include <driver/gpio.h>

// BT API + Bluedroid
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
// GAP API
#include "esp_gap_ble_api.h"
// GATT defines and server API
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

// nvs
#include "nvs_flash.h"

// logging
#include "esp_log.h"

// tags for logging system
#define GATTS_TAG "GATT debug"
#define GAP_TAG "GAP debug"
#define NVS_TAG "NVS debug"
#define GPIO_TAG "GPIO debug"
#define BLE_GENERAL_TAG "BLE general debug"
#define GENERAL_TAG "GENERAL debug"

// get size of array
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

// sort key pressed to command
static uint8_t keys_map[4][4] = {
		{ 0x0C, 0x0D, 0x0E, 0x0F },
		{ 0x08, 0x09, 0x0A, 0x0B },
		{ 0x04, 0x05, 0x06, 0x07 },
		{ 0x00, 0x01, 0x02, 0x03 },
};

// the GPIO arrays (use your esp32 gpio matrix buttons here)
static gpio_num_t output_pins[] = { GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_4, GPIO_NUM_16 };
static gpio_num_t input_pins[] = { GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21 };

// the length of the GPIO pins arrays
const static size_t output_pins_len = ARRAY_SIZE(output_pins);
const static size_t input_pins_len = ARRAY_SIZE(input_pins);

// GAP service defines
#define DEVICE_NAME "esp32-remote-controller"

#define NUM_HANDLES 4

// GAP service info structs
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

// your adv_service_uuid_here
static uint8_t adv_service_uuid128[16] = {

};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    // you can use your own apperance profile here
    .appearance = ESP_BLE_APPEARANCE_GENERIC_REMOTE,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = sizeof(uint8_t),
    .p_service_data = NULL,
    .service_uuid_len = ARRAY_SIZE(adv_service_uuid128),
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
    // you can use your own apperance profile here
    .appearance = ESP_BLE_APPEARANCE_GENERIC_REMOTE,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = sizeof(uint8_t),
    .p_service_data = NULL,
    .service_uuid_len = ARRAY_SIZE(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

// profile keeper for important callback variables
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

void gap_callback_event_handler (esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gatt_callback_event_handler (esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab = {
        .gatts_cb = gatt_callback_event_handler,                   /* This demo does not implement, similar as profile A */
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		.service_handle = ESP_GATT_IF_NONE,
		.char_handle = ESP_GATT_IF_NONE,
};

static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   char *key_str = NULL;
   switch(key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

   }

   return key_str;
}


// callback event handler for gap events
void gap_callback_event_handler (esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event)
	{
	// key event for paired
	case ESP_GAP_BLE_KEY_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_KEY_EVT");
        ESP_LOGI(GAP_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
	// numeric Numeric Comparison request event
	case ESP_GAP_BLE_NC_REQ_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_NC_REQ_EVT");
		esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
		break;
	// authentication complete indication
	case ESP_GAP_BLE_AUTH_CMPL_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_AUTH_CMPL_EVT");
		if (param->ble_security.auth_cmpl.success == true) {
			ESP_LOGI(GAP_TAG, "Successfully authenticated");
		} else {
			ESP_LOGI(GAP_TAG, "Failed to authenticate, HCI reason / error code: %u", param->ble_security.auth_cmpl.fail_reason);
		}
		break;
	// BLE security request
	case ESP_GAP_BLE_SEC_REQ_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_SEC_REQ_EVT");
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		break;
	// after setting the adv data
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
		adv_config_done &= (~adv_config_flag);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
			ESP_LOGI(GAP_TAG, "Started advertising");
		}
		break;
	// after setting the scan response data
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
		adv_config_done &= (~scan_rsp_config_flag);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
			ESP_LOGI(GAP_TAG, "Started advertising");
		}
		break;
	// starting the adv
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGI(GAP_TAG, "Failed to start advertising");
		}
		else {
			ESP_LOGI(GAP_TAG, "Successfully started advertising");
		}
		break;
	// stoping the adv
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGI(GAP_TAG, "Failed to stop advertising");
		}
		else {
			ESP_LOGI(GAP_TAG, "Successfully stopped advertising");
		}
		break;
	// updated the connection params
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(GAP_TAG, "EVENT: ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
        ESP_LOGI(GAP_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
		break;
	default:
		ESP_LOGI(GAP_TAG, "GAP callback event: %d\n", event);
		break;
	}
	return;
}

// the characteristics uuid to add
static esp_bt_uuid_t char_uuid = {
		.len = ESP_UUID_LEN_128,
		.uuid.uuid128 = {
    // your uuid here
		},
};

static uint8_t command = 0xFF;

static esp_attr_value_t char_val = {
		.attr_max_len = ESP_UUID_LEN_16,
		.attr_len = sizeof(uint8_t),
		.attr_value = &command,
};

// the CCCD uuid to add - notifications
static esp_bt_uuid_t cccd_uuid = {
		.len = ESP_UUID_LEN_16,
		.uuid.uuid16 = 0x2902,
};

// little endian, first bit for notifications
static uint8_t cccd_bits[] = { 0x01, 0x00 };

static esp_attr_value_t cccd_val = {
		.attr_max_len = ESP_UUID_LEN_16,
		.attr_len = ESP_UUID_LEN_16,
		.attr_value = cccd_bits,
};

static esp_attr_control_t cccd_ctrl = {
		.auto_rsp = ESP_GATT_RSP_BY_APP,
};

// check if noti are enabled
static bool notify_enable = false;

// callback event handler for gatt events
void gatt_callback_event_handler (esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	esp_gatt_rsp_t rsp;
	switch (event)
	{
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
		break;
	// create table completed
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_CREAT_ATTR_TAB_EVT");
		break;
	// send response complete
	case ESP_GATTS_RESPONSE_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_RESPONSE_EVT");
		break;
	// listening to connect (prob after open)
	case ESP_GATTS_LISTEN_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_LISTEN_EVT");
		break;
	// gatt server close
	case ESP_GATTS_CLOSE_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_CLOSE_EVT");
		break;
	// disconnect from peer
	case ESP_GATTS_CANCEL_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_CANCEL_OPEN_EVT");
		break;
	// connect to peer
	case ESP_GATTS_OPEN_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_OPEN_EVT");
		break;
	// gatt client disconnect
	case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        gl_profile_tab.gatts_if = ESP_GATT_IF_NONE;
		break;
	// gatt client connect (update conn params)
	case ESP_GATTS_CONNECT_EVT:
		esp_ble_conn_update_params_t conn_params = {0};
		memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
		/* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
		conn_params.latency = 0;
		conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
		conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
		conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
				 param->connect.conn_id,
				 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
				 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
		gl_profile_tab.conn_id = param->connect.conn_id;

        gl_profile_tab.gatts_if = gatts_if;
		// start sent the update connection parameters to the peer device.
		esp_ble_gap_update_conn_params(&conn_params);

		break;
	// stop service
	case ESP_GATTS_STOP_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_STOP_EVT");
		break;
	// when recieve confirm
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_CONF_EVT");
		break;
	// when a client wants to read
	case ESP_GATTS_READ_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_READ_EVT");
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = *(char_val.attr_value);
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
		break;
	// registering the app
	case ESP_GATTS_REG_EVT:
		if (param->reg.status == ESP_GATT_OK) {
	        ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_REG_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);

	        gl_profile_tab.service_id.is_primary = true;
	        gl_profile_tab.service_id.id.inst_id = 0x00;
	        gl_profile_tab.service_id.id.uuid.len = ESP_UUID_LEN_128;

	        static uint8_t srvc_uuid128[16] = {
	        	    // your service uuid here 
	        };

	        memcpy(gl_profile_tab.service_id.id.uuid.uuid.uuid128, srvc_uuid128, ESP_UUID_LEN_128);

	        esp_err_t set_dev_name_ret = esp_bt_dev_set_device_name(DEVICE_NAME);
	        if (set_dev_name_ret){
	            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
	        }

	        //config adv data
			esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
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

	        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab.service_id, NUM_HANDLES);
		}
		break;
	// when creating service
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_CREATE_EVT");
		if (param->create.status == ESP_GATT_OK) {

			gl_profile_tab.service_handle = param->create.service_handle;
			esp_ble_gatts_start_service(gl_profile_tab.service_handle);
		}
		break;
	// start service
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_START_EVT");
		if (param->start.status == ESP_GATT_OK) {
	        gl_profile_tab.service_handle = param->start.service_handle;
			esp_ble_gatts_add_char(gl_profile_tab.service_handle, &char_uuid, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &char_val, NULL);
		}
		break;
	// adding char to service
	case ESP_GATTS_ADD_CHAR_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_ADD_CHAR_EVT");
		if (param->add_char.status == ESP_GATT_OK) {
			gl_profile_tab.char_handle = param->add_char.attr_handle;
			esp_ble_gatts_add_char_descr(gl_profile_tab.service_handle, &cccd_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &cccd_val, &cccd_ctrl);
		}
		break;
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_ADD_CHAR_DESCR_EVT");
		if (param->add_char_descr.status == ESP_GATT_OK) {
			ESP_LOGI(GATTS_TAG, "CCCD added successfully");
		} else {
			ESP_LOGI(GATTS_TAG, "Adding CCCD failed, error code: %x", param->add_char_descr.status);
		}
		break;
	case ESP_GATTS_SET_ATTR_VAL_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_SET_ATTR_VAL_EVT");
		break;
	case ESP_GATTS_WRITE_EVT:
		ESP_LOGI(GATTS_TAG, "EVENT: ESP_GATTS_WRITE_EVT");
        if (param->write.value[0] & 1) {
        	notify_enable = true;
        } else {
        	notify_enable = false;
        }
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->write.handle;
        rsp.attr_value.len = 0;
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                    ESP_GATT_OK, &rsp);
		break;
	// all other non needed
	default:
		ESP_LOGI(GATTS_TAG, "GATT callback event: %d\n", event);
		break;
	}
	return;
}

void setup_matrix_pins()
{
	esp_err_t status;

	for (size_t i = 0; i < output_pins_len; ++i)
	{
		status = gpio_set_direction(output_pins[i], GPIO_MODE_OUTPUT);
		ESP_ERROR_CHECK(status);
	}

	for (size_t i = 0; i < input_pins_len; ++i)
	{
		status = gpio_set_direction(input_pins[i], GPIO_MODE_INPUT);
		ESP_ERROR_CHECK(status);

		status = gpio_pullup_en(input_pins[i]);
		ESP_ERROR_CHECK(status);
	}

	return;
}

void set_rows_output_high()
{
	esp_err_t status;

	for (size_t i = 0; i < output_pins_len; ++i)
	{
		status = gpio_set_level(output_pins[i], 1);
		ESP_ERROR_CHECK(status);
	}

	return;
}


// half a second for delay after printing (remove duplicates)
#define DELAY_MS 500

void app_main(void)
{
	esp_err_t status;

	/*
	 * NVS setup
	 */

	status = nvs_flash_init();
	if (status) {
		ESP_LOGE(NVS_TAG, "%s initialize NVS flash failed: %s\n", __func__, esp_err_to_name(status));
		return;
	}

	/*
	 * BLE setups
	 */

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	status = esp_bt_controller_init(&bt_cfg);
	if (status) {
		ESP_LOGE(BLE_GENERAL_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(status));
		return;
	}

	status = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (status) {
		ESP_LOGE(BLE_GENERAL_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(status));
		return;
	}
	status = esp_bluedroid_init();
	if (status) {
		ESP_LOGE(BLE_GENERAL_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(status));
		return;
	}
	status = esp_bluedroid_enable();
	if (status) {
		ESP_LOGE(BLE_GENERAL_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(status));
		return;
	}

	// GATT server callbacks for GATT events
	status = esp_ble_gatts_register_callback(gatt_callback_event_handler);
	if (status) {
		ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", status);
		return;
	}

	status = esp_ble_gap_register_callback(gap_callback_event_handler);
	if (status){
		ESP_LOGE(GAP_TAG, "gap register error, error code = %x", status);
		return;
	}

	status = esp_ble_gatts_app_register(0);
	if (status){
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", status);
		return;
	}

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }


    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));

	/*
	 * GPIO matrix setups
	 */

	// set all GPIO pins to their needed type - input pullup / output
	setup_matrix_pins();

	// set all row-pins to high - matrix module
	set_rows_output_high();

	ESP_LOGI(GPIO_TAG, "All GPIO initializing is done");

	/*
	 * main loop running
	 */

	while (true)
	{

		for (size_t i = 0; i < output_pins_len; ++i)
		{
			// set current row-pin to low
			status = gpio_set_level(output_pins[i], 0);
			ESP_ERROR_CHECK(status);

			//read the state of the each column
			for (size_t j = 0; j < input_pins_len; ++j)
			{
				// get the state
				const int state = gpio_get_level(input_pins[j]);

				// if it's low - the key at (i, j) is pressed
				if (state == 0)
				{
					// Here, send the BLE command accordingly to key pressed

					char_val.attr_value = &keys_map[i][j];
					ESP_LOGI(GENERAL_TAG, "Key %u is pressed!\n", *char_val.attr_value);

					if ((gl_profile_tab.gatts_if != ESP_GATT_IF_NONE) && (gl_profile_tab.char_handle != ESP_GATT_IF_NONE) && (notify_enable)) {
						// notify the client for change command
						esp_ble_gatts_set_attr_value(gl_profile_tab.char_handle, char_val.attr_len, char_val.attr_value);

						status = esp_ble_gatts_send_indicate(gl_profile_tab.gatts_if, gl_profile_tab.conn_id, gl_profile_tab.char_handle, char_val.attr_len, char_val.attr_value, false);
						ESP_ERROR_CHECK(status);

						ESP_LOGI(GATTS_TAG, "Set the command's value to: %d", *char_val.attr_value);
					}

					vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
				}
			}

			// restore current row-pin to high
			status = gpio_set_level(output_pins[i], 1);
			ESP_ERROR_CHECK(status);
		}
	}

	return;
}
