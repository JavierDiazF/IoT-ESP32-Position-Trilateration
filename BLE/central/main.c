/*
 * SPDX-FileCopyrightText: 2017-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include "esp_log.h"
#include "nvs_flash.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_prox_cent.h"

#define N 2.0 // Variable de entorno, de 2 a 4, de menor a mayor obstáculos
#define MEASURED_PWR -75 // Medido a 1 metro de distancia. Repetir cada vez que se haga

struct ble_id_distance_t{
	char identificador[8];
	uint16_t distancia_handler;
	double distancia;
	double coordx;
	double coordy;
};

struct packet{
	char id[8];
	double coordx;
	double coordy;
};

static const char *tag = "BLE_CENTRAL";
static uint8_t peer_addr[6];
static uint8_t link_supervision_timeout;
static int8_t tx_pwr_lvl;
static struct ble_prox_cent_conn_peer conn_peer[MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 1];
static struct ble_prox_cent_link_lost_peer disconn_peer[MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 1];
// PARA ID Y DISTANCIA
static struct ble_id_distance_t id_dist_conn_peer[MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 1];
static int num_clientes;
static double centcoordx;
static double centcoordy;
//static char identificador_ble[8];

/* Note: Path loss is calculated using formula : threshold - RSSI value
 *       by default threshold is kept -128 as per the spec
 *       high_threshold and low_threshold are hardcoded after testing and noting
 *       RSSI values when distance between devices are less and more.
 */
static int8_t high_threshold = -70;
static int8_t low_threshold = -100;

// Definimos los uuid-128 de los servicios que vamos a crear
static const ble_uuid128_t servicio_uuid = BLE_UUID128_INIT(0x01, 0x96, 0x43, 0x35, 0x6c, 0x13, 0x7b, 0xb3, 0x9b, 0x1d, 0xf6, 0x7a, 0x54, 0xfc, 0xf5, 0x9f);
static const ble_uuid128_t id_chr_uuid = BLE_UUID128_INIT(0x01, 0x96, 0x43, 0x35, 0x6c, 0x13, 0x7a, 0x5e, 0x83, 0x3c, 0x61, 0x6e, 0xb2, 0xa9, 0x7b, 0xc6);
static const ble_uuid128_t distancia_chr_uuid = BLE_UUID128_INIT(0x01, 0x96, 0x43, 0x35, 0x6c, 0x13, 0x79, 0x2a, 0x9e, 0x83, 0x0e, 0xbc, 0x43, 0x08, 0x3e, 0x96);

void ble_store_config_init(void);
static void ble_prox_cent_scan(void);
static int ble_prox_cent_gap_event(struct ble_gap_event *event, void *arg);

void calcula_posicion_task(void *pvParameters){
        float A, B, C, D, E, F, denominador;
        // Esto es por las pruebas con 2 esp32. Cuando tenga 3 se puede quitar
	/*
        if (num_clientes == 1){
                id_dist_conn_peer[num_clientes+1].coordx = 20.0;
                id_dist_conn_peer[num_clientes+1].coordy = 5.0;
                id_dist_conn_peer[num_clientes+1].distancia = 100.0;
        }
	*/
        A = 2*(id_dist_conn_peer[0].coordx - id_dist_conn_peer[1].coordx); //2*(x_1 - x_2)
        B = 2*(id_dist_conn_peer[0].coordy - id_dist_conn_peer[1].coordy); //2*(y_1 - y_2)
        C = pow(id_dist_conn_peer[0].distancia,2) - pow(id_dist_conn_peer[1].distancia,2) - pow(id_dist_conn_peer[0].coordx,2) + pow(id_dist_conn_peer[1].coordx,2) - pow(id_dist_conn_peer[0].coordy,2) + pow(id_dist_conn_peer[1].coordy,2);

        D = 2*(id_dist_conn_peer[0].coordx - id_dist_conn_peer[2].coordx); //2*(x_1 - x_3)
        E = 2*(id_dist_conn_peer[0].coordy - id_dist_conn_peer[2].coordy); //2*(y_1 - y_3)
        F = pow(id_dist_conn_peer[0].distancia,2) - pow(id_dist_conn_peer[2].distancia,2) - pow(id_dist_conn_peer[0].coordx,2) + pow(id_dist_conn_peer[2].coordx,2) - pow(id_dist_conn_peer[0].coordy,2) + pow(id_dist_conn_peer[2].coordy,2);
        denominador = (A*E-B*D);
        if(fabs(denominador)< 1e-6){
                MODLOG_DFLT(ERROR, "No se puede resolver (división por cero)");
                vTaskDelete(NULL);
                return;
        }
        centcoordx = (C*E-F*B)/denominador;
        centcoordy = (A*F-D*C)/denominador;
        MODLOG_DFLT(INFO, "Coordenadas x: %g, y: %g", centcoordx, centcoordy);

        vTaskDelete(NULL);
}

static int
ble_prox_cent_on_read(uint16_t conn_handle,
                      const struct ble_gatt_error *error,
                      struct ble_gatt_attr *attr,
                     void *arg)
{
    MODLOG_DFLT(INFO, "Read on tx power level char completed; status=%d "
                "conn_handle=%d\n",
                error->status, conn_handle);
    if (error->status == 0) {
        MODLOG_DFLT(INFO, " attr_handle=%d value=", attr->handle);
        print_mbuf(attr->om);
        os_mbuf_copydata(attr->om, 0, attr->om->om_len, &tx_pwr_lvl);
        conn_peer[conn_handle].calc_path_loss = true;
    }

    return 0;
}

// Y AQUÍ ME HAGO EL CÓDIGO PARA LEER LAS CARACTERÍSTICAS QUE HE CREADO
static int ble_id_cent_read(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg){
	MODLOG_DFLT(INFO, "Leyendo el id del sensor; status=%d, conn_handle=%d", error->status, conn_handle);
	if (error->status == 0){
		char rx_buffer[sizeof(struct packet)];
		struct packet *rx_pkt = (struct packet*) rx_buffer;
		MODLOG_DFLT(INFO, " attr_handle=%d value=", attr->handle);
		//print_mbuf(attr->om);
		os_mbuf_copydata(attr->om, 0, attr->om->om_len, &rx_buffer);
		strncpy(id_dist_conn_peer[conn_handle].identificador, rx_pkt->id, sizeof(rx_pkt->id));
		id_dist_conn_peer[conn_handle].coordx = rx_pkt->coordx;
		id_dist_conn_peer[conn_handle].coordy = rx_pkt->coordy;
		MODLOG_DFLT(INFO, "Guardado sensor %s en coordenadas: %g, %g", id_dist_conn_peer[conn_handle].identificador, id_dist_conn_peer[conn_handle].coordx, id_dist_conn_peer[conn_handle].coordy);
	}
	return 0;
}
/**
 * Application callback.  Called when the write of alert level char
 * characteristic has completed.
 */
static int
ble_prox_cent_on_write(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr,
                       void *arg)
{
    MODLOG_DFLT(INFO, "Write alert level char completed; status=%d conn_handle=%d",
                error->status, conn_handle);

    /* Read Tx Power level characteristic. */
    const struct peer_chr *chr;
    int rc;
    const struct peer *peer = peer_find(conn_handle);

    chr = peer_chr_find_uuid(peer,
                             BLE_UUID16_DECLARE(BLE_SVC_TX_POWER_UUID16),
                             BLE_UUID16_DECLARE(BLE_SVC_PROX_CHR_UUID16_TX_PWR_LVL));
    if (chr == NULL) {
        MODLOG_DFLT(ERROR, "Error: Peer doesn't support the"
                    "Tx power level characteristic\n");
        goto err;
    }

    rc = ble_gattc_read(conn_handle, chr->chr.val_handle,
                        ble_prox_cent_on_read, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error: Failed to read characteristic; rc=%d\n",
                    rc);
        goto err;
    }

    // AHORA VOY A BUSCCAR LOS HANDLERS QUE HE CREADO ID
    chr = peer_chr_find_uuid(peer, &servicio_uuid.u, &id_chr_uuid.u);
    if (chr == NULL) {
	    MODLOG_DFLT(ERROR, "Error al encontrar la característicca ID");
	    goto err;
    }
    rc = ble_gattc_read(conn_handle, chr->chr.val_handle, ble_id_cent_read, NULL);
    if (rc != 0){
	    MODLOG_DFLT(ERROR, "Error al leer el ID rc=%d", rc);
	    goto err;
    }
    return 0;
err:
    /* Terminate the connection. */
    return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}
/**
 * Performs following GATT operations against the specified peer:
 * 1. Writes the alert level characteristic.
 * 2. After write is completed, it reads the Tx Power Level characteristic.
 *
 * If the peer does not support a required service, characteristic, or
 * descriptor, then the peer lied when it claimed support for the link
 * loss service!  When this happens, or if a GATT procedure fails,
 * this function immediately terminates the connection.
 */
static void
ble_prox_cent_read_write_subscribe(const struct peer *peer)
{
    const struct peer_chr *chr;
    int rc;

    /* Storing the val handle of immediate alert characteristic */
    chr = peer_chr_find_uuid(peer,
                             BLE_UUID16_DECLARE(BLE_SVC_IMMEDIATE_ALERT_UUID16),
                             BLE_UUID16_DECLARE(BLE_SVC_PROX_CHR_UUID16_ALERT_LVL));
    if (chr != NULL) {
        conn_peer[peer->conn_handle].val_handle = chr->chr.val_handle;
    } else {
        MODLOG_DFLT(ERROR, "Error: Peer doesn't support the alert level"
                    " characteristic of immediate alert loss service\n");
    }

    /* Write alert level characteristic. */
    chr = peer_chr_find_uuid(peer,
                             BLE_UUID16_DECLARE(BLE_SVC_LINK_LOSS_UUID16),
                             BLE_UUID16_DECLARE(BLE_SVC_PROX_CHR_UUID16_ALERT_LVL));
    if (chr == NULL) {
        MODLOG_DFLT(ERROR, "Error: Peer doesn't support the alert level"
                    " characteristic\n");
        goto err;
    }

    rc = ble_gattc_write_flat(peer->conn_handle, chr->chr.val_handle,
                              &link_supervision_timeout, sizeof(link_supervision_timeout),
                              ble_prox_cent_on_write, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error: Failed to write characteristic; rc=%d\n",
                    rc);
        goto err;
    }
     // Ahora escribo la distancia
    chr = peer_chr_find_uuid(peer, &servicio_uuid.u, &distancia_chr_uuid.u);
    if (chr == NULL){
	    MODLOG_DFLT(ERROR, "Error el periférico no tiene el servicio de distancia");
	    goto err;
    } else id_dist_conn_peer[peer->conn_handle].distancia_handler = chr->chr.val_handle;
    //rc = ble_gattc_write_flat(peer->conn_handle, chr->chr.val_handle, &link_supervision_timeout, sizeof(link_supervision_timeout), NULL, NULL);

    return;
err:
    /* Terminate the connection. */
    ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * Called when service discovery of the specified peer has completed.
 */
static void
ble_prox_cent_on_disc_complete(const struct peer *peer, int status, void *arg)
{

    if (status != 0) {
        /* Service discovery failed.  Terminate the connection. */
        MODLOG_DFLT(ERROR, "Error: Service discovery failed; status=%d "
                    "conn_handle=%d\n", status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    /* Service discovery has completed successfully.  Now we have a complete
     * list of services, characteristics, and descriptors that the peer
     * supports.
     */
    MODLOG_DFLT(INFO, "Service discovery complete; status=%d "
                "conn_handle=%d\n", status, peer->conn_handle);

    /* Now perform GATT procedures against the peer: read,
     * write.
     */
    ble_prox_cent_read_write_subscribe(peer);
}

/**
 * Initiates the GAP general discovery procedure.
 */
static void
ble_prox_cent_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      ble_prox_cent_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
}

/**
 * Indicates whether we should try to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and support for the Health Thermometer service.
 */

#if CONFIG_EXAMPLE_EXTENDED_ADV
static int
ext_ble_prox_cent_should_connect(const struct ble_gap_ext_disc_desc *disc)
{
    int offset = 0;
    int ad_struct_len = 0;

    if (disc->legacy_event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
            disc->legacy_event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        return 0;
    }
    if (strlen(CONFIG_EXAMPLE_PEER_ADDR) && (strncmp(CONFIG_EXAMPLE_PEER_ADDR, "ADDR_ANY", strlen    ("ADDR_ANY")) != 0)) {
        ESP_LOGI(tag, "Peer address from menuconfig: %s", CONFIG_EXAMPLE_PEER_ADDR);
        /* Convert string to address */
        sscanf(CONFIG_EXAMPLE_PEER_ADDR, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &peer_addr[5], &peer_addr[4], &peer_addr[3],
               &peer_addr[2], &peer_addr[1], &peer_addr[0]);
        if (memcmp(peer_addr, disc->addr.val, sizeof(disc->addr.val)) != 0) {
            return 0;
        }
    }

    /* The device has to advertise support for Proximity sensor (link loss)
    * service (0x1803).
    */
    do {
        ad_struct_len = disc->data[offset];

        if (!ad_struct_len) {
            break;
        }

        /* Search if Proximity Sensor (Link loss) UUID is advertised */
        if (disc->data[offset] == 0x03 && disc->data[offset + 1] == 0x03) {
            if ( disc->data[offset + 2] == 0x18 && disc->data[offset + 3] == 0x03 ) {
                return 1;
            }
        }

        offset += ad_struct_len + 1;

    } while ( offset < disc->length_data );

    return 0;
}
#else

static int
ble_prox_cent_should_connect(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    int rc;
    int i;

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
            disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {

        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        return rc;
    }

    if (strlen(CONFIG_EXAMPLE_PEER_ADDR) && (strncmp(CONFIG_EXAMPLE_PEER_ADDR, "ADDR_ANY", strlen("ADDR_ANY")) != 0)) {
        ESP_LOGI(tag, "Peer address from menuconfig: %s", CONFIG_EXAMPLE_PEER_ADDR);
        /* Convert string to address */
        sscanf(CONFIG_EXAMPLE_PEER_ADDR, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &peer_addr[5], &peer_addr[4], &peer_addr[3],
               &peer_addr[2], &peer_addr[1], &peer_addr[0]);
        if (memcmp(peer_addr, disc->addr.val, sizeof(disc->addr.val)) != 0) {
            return 0;
        }
    }

    /* The device has to advertise support for the Proximity sensor (link loss)
     * service (0x1803).
     */
    for (i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == BLE_SVC_LINK_LOSS_UUID16) {
            return 1;
        }
    }

    return 0;
}
#endif

/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Proximity Sensor service.
 */
static void
ble_prox_cent_connect_if_interesting(void *disc)
{
    uint8_t own_addr_type;
    int rc;
    ble_addr_t *addr;

    /* Don't do anything if we don't care about this advertiser. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
    if (!ext_ble_prox_cent_should_connect((struct ble_gap_ext_disc_desc *)disc)) {
        return;
    }
#else
    if (!ble_prox_cent_should_connect((struct ble_gap_disc_desc *)disc)) {
        return;
    }
#endif

#if !(MYNEWT_VAL(BLE_HOST_ALLOW_CONNECT_WITH_SCAN))
    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        MODLOG_DFLT(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }
#endif

    /* Figure out address to use for connect (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
     * timeout.
     */
#if CONFIG_EXAMPLE_EXTENDED_ADV
    addr = &((struct ble_gap_ext_disc_desc *)disc)->addr;
#else
    addr = &((struct ble_gap_disc_desc *)disc)->addr;
#endif
    rc = ble_gap_connect(own_addr_type, addr, 30000, NULL,
                         ble_prox_cent_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error: Failed to connect to device; addr_type=%d "
                    "addr=%s; rc=%d\n",
                    addr->type, addr_str(addr->val), rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  ble_prox_cent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                              ble_prox_cent.
 *
 * @return                      0 if the application successfully handled the
 *                              event; nonzero on failure.  The semantics
 *                              of the return code is specific to the
 *                              particular GAP event being signalled.
 */
static int
ble_prox_cent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
        if (rc != 0) {
            return 0;
        }

        /* An advertisement report was received during GAP discovery. */
        print_adv_fields(&fields);

        /* Try to connect to the advertiser if it looks interesting. */
        ble_prox_cent_connect_if_interesting(&event->disc);
        return 0;

    case BLE_GAP_EVENT_LINK_ESTAB:
        /* A new connection was established or a connection attempt failed. */
        if (event->link_estab.status == 0) {
            /* Connection successfully established. */
            MODLOG_DFLT(INFO, "Connection established ");

            rc = ble_gap_conn_find(event->link_estab.conn_handle, &desc);
            assert(rc == 0);
            print_conn_desc(&desc);
            MODLOG_DFLT(INFO, "\n");

            link_supervision_timeout = 8 * desc.conn_itvl;

            /* Remember peer. */
            rc = peer_add(event->link_estab.conn_handle);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "Failed to add peer; rc=%d\n", rc);
                return 0;
            }

            /* Check if this device is reconnected */
            for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
                if (disconn_peer[i].addr != NULL) {
                    if (memcmp(disconn_peer[i].addr, &desc.peer_id_addr.val, BLE_ADDR_LEN)) {
                        /* Peer reconnected. Stop alert for this peer */
                        free(disconn_peer[i].addr);
                        disconn_peer[i].addr = NULL;
                        disconn_peer[i].link_lost = false;
                        break;
                    }
                }
            }

#if CONFIG_EXAMPLE_ENCRYPTION
            /** Initiate security - It will perform
             * Pairing (Exchange keys)
             * Bonding (Store keys)
             * Encryption (Enable encryption)
             * Will invoke event BLE_GAP_EVENT_ENC_CHANGE
             **/
            rc = ble_gap_security_initiate(event->link_estab.conn_handle);
            if (rc != 0) {
                MODLOG_DFLT(INFO, "Security could not be initiated, rc = %d\n", rc);
                return ble_gap_terminate(event->link_estab.conn_handle,
                                         BLE_ERR_REM_USER_CONN_TERM);
            } else {
                MODLOG_DFLT(INFO, "Connection secured\n");
            }
#else
            /* Perform service discovery */
            rc = peer_disc_all(event->link_estab.conn_handle,
                               ble_prox_cent_on_disc_complete, NULL);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
                return 0;
            }
#endif
        } else {
            /* Connection attempt failed; resume scanning. */
            MODLOG_DFLT(ERROR, "Error: Connection failed; status=%d\n",
                        event->link_estab.status);
        }
        ble_prox_cent_scan();
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");

        /* Start the link loss alert for this connection handle */
        for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
            if (disconn_peer[i].addr == NULL) {
                disconn_peer[i].addr = (uint8_t *)malloc(BLE_ADDR_LEN * sizeof(uint8_t));
                if (disconn_peer[i].addr == NULL) {
                    return BLE_HS_ENOMEM;
                }
                memcpy(disconn_peer[i].addr, &event->disconnect.conn.peer_id_addr.val,
                       BLE_ADDR_LEN);
                disconn_peer[i].link_lost = true;
                break;
            }
        }
        /* Stop calculating path loss, restart once connection is established again */
        conn_peer[event->disconnect.conn.conn_handle].calc_path_loss = false;
        conn_peer[event->disconnect.conn.conn_handle].val_handle = 0;

        /* Forget about peer. */
        peer_delete(event->disconnect.conn.conn_handle);

        /* Resume scanning. */
        ble_prox_cent_scan();
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        MODLOG_DFLT(INFO, "discovery complete; reason=%d\n",
                    event->disc_complete.reason);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        MODLOG_DFLT(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
#if CONFIG_EXAMPLE_ENCRYPTION
        /*** Go for service discovery after encryption has been successfully enabled ***/
        rc = peer_disc_all(event->link_estab.conn_handle,
                           ble_prox_cent_on_disc_complete, NULL);
        if (rc != 0) {
            MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
            return 0;
        }
#endif
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        /* Peer sent us a notification or indication. */
        MODLOG_DFLT(INFO, "received %s; conn_handle=%d attr_handle=%d "
                    "attr_len=%d\n",
                    event->notify_rx.indication ?
                    "indication" :
                    "notification",
                    event->notify_rx.conn_handle,
                    event->notify_rx.attr_handle,
                    OS_MBUF_PKTLEN(event->notify_rx.om));

        /* Attribute data is contained in event->notify_rx.om. Use
         * `os_mbuf_copydata` to copy the data received in notification mbuf */
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

#if CONFIG_EXAMPLE_EXTENDED_ADV
    case BLE_GAP_EVENT_EXT_DISC:
        /* An advertisement report was received during GAP discovery. */
        ext_print_adv_report(&event->disc);

        ble_prox_cent_connect_if_interesting(&event->disc);
        return 0;
#endif

    default:
        return 0;
    }
}

void
ble_prox_cent_path_loss_task(void *pvParameters)
{
    int8_t rssi;
    int rc;
    int path_loss;

    while (1) {
        for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
            if (conn_peer[i].calc_path_loss) {
                MODLOG_DFLT(INFO, "Connection handle : %d", i);
                rc = ble_gap_conn_rssi(i, &rssi);
                if (rc == 0) {
                    MODLOG_DFLT(INFO, "Current RSSI = %d", rssi);
                } else {
                    MODLOG_DFLT(ERROR, "Failed to get current RSSI");
                }

                path_loss = tx_pwr_lvl - rssi;
                MODLOG_DFLT(INFO, "path loss = %d pwr lvl = %d rssi = %d", path_loss, tx_pwr_lvl, rssi);
		id_dist_conn_peer[i].distancia = pow(10.0, ((float)(MEASURED_PWR - rssi)) / (10.0*N));
		MODLOG_DFLT(INFO, "Distancia al sensor %s: %g", id_dist_conn_peer[i].identificador, id_dist_conn_peer[i].distancia);
		rc = ble_gattc_write_no_rsp_flat(i, id_dist_conn_peer[i].distancia_handler, &id_dist_conn_peer[i].distancia, sizeof(id_dist_conn_peer[i].distancia));
		if (rc != 0){
			MODLOG_DFLT(ERROR, "Error al escribir la distancia; rc=%d", rc);
		} else {
			MODLOG_DFLT(INFO, "Escritura de la distancia hecha");
		}

                if ((conn_peer[i].val_handle != 0) &&
                        (path_loss > high_threshold || path_loss < low_threshold)) {

                    if (path_loss < low_threshold) {
                        path_loss = 0;
                    }

                    rc = ble_gattc_write_no_rsp_flat(i, conn_peer[i].val_handle,
                                                     &path_loss, sizeof(path_loss));
                    if (rc != 0) {
                        MODLOG_DFLT(ERROR, "Error: Failed to write characteristic; rc=%d\n",
                                    rc);
                    } else {
                        MODLOG_DFLT(INFO, "Write to alert level characteristis done");
                    }
                }
	    	if (i == 2){ //Tengo 3 dispositivos conectados
			num_clientes = i;
			MODLOG_DFLT(INFO, "Se han encontrado %d dispositivos. Se procede a calcular la posicion", num_clientes+1);
			xTaskCreate(calcula_posicion_task, "ble_central", 4096, NULL, 5, NULL);
	    	}
            }
        }
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void
ble_prox_cent_link_loss_task(void *pvParameters)
{
    while (1) {
        for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
            if (disconn_peer[i].link_lost && disconn_peer[i].addr != NULL) {
                MODLOG_DFLT(INFO, "Link lost for device with conn_handle %d", i);
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void
ble_prox_cent_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
ble_prox_cent_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Begin scanning for a peripheral to connect to. */
    ble_prox_cent_scan();
}

void ble_prox_cent_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static void
ble_prox_cent_init(void)
{
    /* Task for calculating path loss */
    xTaskCreate(ble_prox_cent_path_loss_task, "ble_prox_cent_path_loss_task", 4096, NULL, 10, NULL);

    /* Task for alerting when link is lost */
    xTaskCreate(ble_prox_cent_link_loss_task, "ble_prox_cent_link_loss_task", 4096, NULL, 10, NULL);
    return;
}

void
app_main(void)
{
    int rc;
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if  (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
        return;
    }

    /* Initialize a task to keep checking path loss of the link */
    ble_prox_cent_init();

    for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
        disconn_peer[i].addr = NULL;
        disconn_peer[i].link_lost = true;
    }

    /* Configure the host. */
    ble_hs_cfg.reset_cb = ble_prox_cent_on_reset;
    ble_hs_cfg.sync_cb = ble_prox_cent_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize data structures to track connected peers. */
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("esp32-central");
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(ble_prox_cent_host_task);
}
