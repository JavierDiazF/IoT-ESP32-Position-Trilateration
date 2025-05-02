/*
 * SPDX-FileCopyrightText: 2017-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_prox_prph.h"

#if CONFIG_EXAMPLE_EXTENDED_ADV
static uint8_t ext_adv_pattern_1[] = {
    0x02, 0x01, 0x06,
    0x03, 0x03, 0xab, 0xcd,
    0x03, 0x03, 0x18, 0x03,
    0x13, 0X09, 'n', 'i', 'm', 'b', 'l', 'e', '-', 'p', 'r', 'o', 'x', '-', 'p', 'r', 'p', 'h', '-', 'e',
};
#endif

static const char *tag = "BLE_PERIFERICO";
static const char *identificador = "sensor-2";
static const double coordx = 40.5;
static const double coordy = 3.3;
//static char distancia[10] = {};

struct packet{
	char id[8];
	double coordx;
	double coordy;
};

// Definimos los uuid-128 de los servicios que vamos a crear
static const ble_uuid128_t servicio_uuid = BLE_UUID128_INIT(0x01, 0x96, 0x43, 0x35, 0x6c, 0x13, 0x7b, 0xb3, 0x9b, 0x1d, 0xf6, 0x7a, 0x54, 0xfc, 0xf5, 0x9f);
static const ble_uuid128_t id_chr_uuid = BLE_UUID128_INIT(0x01, 0x96, 0x43, 0x35, 0x6c, 0x13, 0x7a, 0x5e, 0x83, 0x3c, 0x61, 0x6e, 0xb2, 0xa9, 0x7b, 0xc6);
static const ble_uuid128_t distancia_chr_uuid = BLE_UUID128_INIT(0x01, 0x96, 0x43, 0x35, 0x6c, 0x13, 0x79, 0x2a, 0x9e, 0x83, 0x0e, 0xbc, 0x43, 0x08, 0x3e, 0x96);

static int ble_prox_prph_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t ble_prox_prph_addr_type;

/**
 * Utility function to log an array of bytes.
 */
void
print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

#if CONFIG_EXAMPLE_EXTENDED_ADV
/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ext_ble_prox_prph_advertise(void)
{
    struct ble_gap_ext_adv_params params;
    struct os_mbuf *data;
    uint8_t instance = 0;
    int rc;

    /* First check if any instance is already active */
    if (ble_gap_ext_adv_active(instance)) {
        return;
    }

    /* use defaults for non-set params */
    memset (&params, 0, sizeof(params));

    /* enable connectable advertising */
    params.connectable = 1;

    /* advertise using random addr */
    params.own_addr_type = BLE_OWN_ADDR_PUBLIC;

    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_2M;
    params.sid = 1;

    params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MIN;

    /* configure instance 0 */
    rc = ble_gap_ext_adv_configure(instance, &params, NULL,
                                   ble_prox_prph_gap_event, NULL);
    assert (rc == 0);

    /* in this case only scan response is allowed */

    /* get mbuf for scan rsp data */
    data = os_msys_get_pkthdr(sizeof(ext_adv_pattern_1), 0);
    assert(data);

    /* fill mbuf with scan rsp data */
    rc = os_mbuf_append(data, ext_adv_pattern_1, sizeof(ext_adv_pattern_1));
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_data(instance, data);
    assert (rc == 0);

    /* start advertising */
    rc = ble_gap_ext_adv_start(instance, 0, 0);
    assert (rc == 0);
}
#else

static void
ble_prox_prph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)identificador;
    fields.name_len = strlen(identificador);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(BLE_SVC_LINK_LOSS_UUID16)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(ble_prox_prph_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_prox_prph_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}
#endif

static int
ble_prox_prph_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_LINK_ESTAB:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->link_estab.status == 0 ? "established" : "failed",
                    event->link_estab.status);

        if (event->link_estab.status != 0) {
        /* Connection failed, resume advertising */
#if CONFIG_EXAMPLE_EXTENDED_ADV
            ext_ble_prox_prph_advertise();
#else
            ble_prox_prph_advertise();
#endif
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
#if CONFIG_EXAMPLE_EXTENDED_ADV
        ext_ble_prox_prph_advertise();
#else
        ble_prox_prph_advertise();
#endif
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
#if CONFIG_EXAMPLE_EXTENDED_ADV
        ext_ble_prox_prph_advertise();
#else
        ble_prox_prph_advertise();
#endif
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, event->subscribe.attr_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}

static int ble_gatt_svc_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
	const ble_uuid_t *uuid = ctxt->chr->uuid;
	double * distancia;
	char tx_buffer[sizeof(struct packet)];
	struct packet *id_coords = (struct packet*) tx_buffer;

	strncpy(id_coords->id, identificador, strlen(identificador));
	id_coords->coordx = coordx;
	id_coords->coordy = coordy;
	if(ble_uuid_cmp(uuid, &id_chr_uuid.u) == 0){
		MODLOG_DFLT(INFO, "El central quiere leer el id del periférico");
		os_mbuf_append(ctxt->om, tx_buffer, sizeof(tx_buffer));
		return 0;
	} else if (ble_uuid_cmp(uuid, &distancia_chr_uuid.u) == 0){
		MODLOG_DFLT(INFO, "Está intentando modificar la distancia");
		if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
			/*
			int len = ctxt->om->om_len;
			if (len> sizeof(distancia)-1)  len = sizeof(distancia)-1;
			memcpy(distancia, ctxt->om->om_data, len);
			distancia[len] = '\0';
			*/
			distancia = (double*)ctxt->om->om_data;
			MODLOG_DFLT(INFO, "Distancia: %g", *distancia);
		}
		return 0;
	}
	return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def new_ble_svc_gatt_def[] = {
	{
	.type  = BLE_GATT_SVC_TYPE_PRIMARY,
	.uuid = &servicio_uuid.u,
	.characteristics = (struct ble_gatt_chr_def[]){
	{
		// Característica ID
		.uuid = &id_chr_uuid.u,
		.access_cb = ble_gatt_svc_handler,
		.flags = BLE_GATT_CHR_F_READ,
	 },
	{
		// Característica distancia
		.uuid = &distancia_chr_uuid.u,
		.access_cb = ble_gatt_svc_handler,
		.flags = BLE_GATT_CHR_F_WRITE,
	},{0},
	},
	},{0},	
};

static void
ble_prox_prph_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &ble_prox_prph_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(ble_prox_prph_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    /* Begin advertising */
#if CONFIG_EXAMPLE_EXTENDED_ADV
    ext_ble_prox_prph_advertise();
#else
    ble_prox_prph_advertise();
#endif
}

static void
ble_prox_prph_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void ble_prox_prph_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

int gatt_svr_init(void){
	int rc = 0;
	ble_svc_gap_init();
	ble_svc_gatt_init();

	rc = ble_gatts_count_cfg(new_ble_svc_gatt_def);
	if (rc!=0){
		return rc;
	}
	rc= ble_gatts_add_svcs(new_ble_svc_gatt_def);
	if (rc!=0){
		return rc;
	}
	return 0;
}


void app_main(void)
{
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }
    
    //static int8_t distancia_ble;
    /* Initialize a task to keep checking path loss of the link */
    ble_svc_prox_init();

    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = ble_prox_prph_on_sync;
    ble_hs_cfg.reset_cb = ble_prox_prph_on_reset;

    /* Enable bonding */
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_mitm = 1;

    // Registramos el servicio creado
    rc = gatt_svr_init();
    assert(rc==0);
    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(identificador);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(ble_prox_prph_host_task);
}
