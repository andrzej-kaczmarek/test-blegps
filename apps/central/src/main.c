/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <stdio.h>
#include "sysinit/sysinit.h"
#include "bsp/bsp.h"
#include "os/os.h"
#include "hal/hal_gpio.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"

#define LOCATION_SCALE      10000000

#define LNS_UUID                        0x1819
#define LNS_LOCATION_AND_SPEED_UUID     0x2A67

#define LNS_LAS_FLAG_SPEED                      0x0001
#define LNS_LAS_FLAG_DISTANCE                   0x0002
#define LNS_LAS_FLAG_LOCATION                   0x0004
#define LNS_LAS_FLAG_ELEVATION                  0x0008
#define LNS_LAS_FLAG_HEADING                    0x0010
#define LNS_LAS_FLAG_ROLLING_TIME               0x0020
#define LNS_LAS_FLAG_UTC_TIME                   0x0040
#define LNS_LAS_FLAG_POS_NO                     0x0000
#define LNS_LAS_FLAG_POS_OK                     0x0080
#define LNS_LAS_FLAG_POS_EST                    0x0100
#define LNS_LAS_FLAG_POS_LAST_KNOWN             0x0180
#define LNS_LAS_FLAG_SPD_2D                     0x0000
#define LNS_LAS_FLAG_SPD_3D                     0x0200
#define LNS_LAS_FLAG_ELEVATION_SRC_GPS          0x0000
#define LNS_LAS_FLAG_ELEVATION_SRC_BAROMETRIC   0x0400
#define LNS_LAS_FLAG_ELEVATION_SRC_DATABASE     0x0800
#define LNS_LAS_FLAG_ELEVATION_SRC_OTHER        0x0c00
#define LNS_LAS_FLAG_HEADING_SRC_MOVEMENT       0x0000
#define LNS_LAS_FLAG_HEADING_SRC_COMPASS        0x1000

struct lns_svcdata {
    uint16_t uuid;
    int32_t lat;
    int32_t lon;
} __attribute__((packed));

static struct os_event gpio_ev;

static ble_addr_t peer_addr;

struct discov_info {
    uint16_t start;
    uint16_t end;
};

static void start_scan(void);

static const char *
phy2str(uint8_t phy)
{
    switch (phy) {
    case BLE_HCI_LE_PHY_1M:
        return "1m";
    case BLE_HCI_LE_PHY_2M:
        return "2m";
    case BLE_HCI_LE_PHY_CODED:
        return "coded";
    }

    return "?";
}

static int
adv_parse_cb(const struct ble_hs_adv_field *field, void *arg)
{
    struct ble_gap_ext_disc_desc *dd = arg;
    struct lns_svcdata *lnsd;

    if (field->type != BLE_HS_ADV_TYPE_SVC_DATA_UUID16) {
        return 1;
    }

    if (field->length != 1 + sizeof(*lnsd)) {
        return 1;
    }

    lnsd = (void *)field->value;

    printf("a %s %d %d.%07d %d.%07d\n", phy2str(dd->prim_phy), (int)dd->rssi,
           (int)lnsd->lat / LOCATION_SCALE, (int)lnsd->lat % LOCATION_SCALE,
           (int)lnsd->lon / LOCATION_SCALE, (int)lnsd->lon % LOCATION_SCALE);


    /* Last seen device is our connection target */
    peer_addr = dd->addr;

    return 0;
}


static void
process_notify(uint16_t conn_handle, struct os_mbuf *om)
{
    uint16_t flags;
    int32_t lat, lon;
    uint8_t tx_phy, rx_phy;
    int8_t rssi;

    os_mbuf_copydata(om, 0, sizeof(flags), &flags);
    os_mbuf_adj(om, sizeof(flags));
    flags = le16toh(flags);

    /* Don't care if location is not available */
    if (!(flags & LNS_LAS_FLAG_LOCATION)) {
        return;
    }

    if (flags & LNS_LAS_FLAG_SPEED) {
        os_mbuf_adj(om, 2);
    }

    if (flags & LNS_LAS_FLAG_DISTANCE) {
        os_mbuf_adj(om, 3);
    }

    os_mbuf_copydata(om, 0, sizeof(lat), &lat);
    os_mbuf_copydata(om, 4, sizeof(lon), &lon);

    /* Don't care about remaining fields */

    ble_gap_read_le_phy(conn_handle, &tx_phy, &rx_phy);
    ble_gap_conn_rssi(conn_handle, &rssi);

    printf("c %s %d %d.%07d %d.%07d\n", phy2str(rx_phy), (int)rssi,
           (int)lat / LOCATION_SCALE, (int)lat % LOCATION_SCALE,
           (int)lon / LOCATION_SCALE, (int)lon % LOCATION_SCALE);
}

static int
gattc_attr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
              struct ble_gatt_attr *attr, void *arg)
{
    /* Disconnect on GATT error */
    if (error->status) {
        ble_gap_terminate(conn_handle, BLE_ERR_CONN_TERM_LOCAL);
        return 0;
    }

    printf("# ready\n");

    return 0;
}


static int
gattc_disc_dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                  uint16_t chr_def_handle, const struct ble_gatt_dsc *dsc,
                  void *arg)
{
    struct discov_info *di = arg;
    uint16_t val = htole16(0x0001); /* Notify */
    int rc;

    switch (error->status) {
    case 0:
        if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
            di->start = dsc->handle;
        }
        break;
    case BLE_HS_EDONE:
        if (di->start) {
            rc = ble_gattc_write_flat(conn_handle, di->start, &val, sizeof(val),
                                      gattc_attr_cb, NULL);
            assert(rc == 0);
            break;
        }
        /* no break */
    default:
        /* Disconnect on GATT error */
        ble_gap_terminate(conn_handle, BLE_ERR_CONN_TERM_LOCAL);
        break;
    }

    return 0;
}

static int
gattc_disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                  const struct ble_gatt_chr *chr, void *arg)
{
    struct discov_info *di = arg;
    uint16_t start_handle;
    int rc;

    switch (error->status) {
    case 0:
        /*
         * If we already have start handle just check if this is following
         * characteristic and end handle. In other case check if this is
         * characteristic we're looking for to set start handle.
         */
        if (di->start && chr->def_handle < di->end) {
            di->end = chr->def_handle - 1;
        } else if (ble_uuid_u16(&chr->uuid.u) == LNS_LOCATION_AND_SPEED_UUID) {
            di->start = chr->val_handle;
        }
        break;
    case BLE_HS_EDONE:
        if (di->start) {
            start_handle = di->start;
            di->start = 0;

            rc = ble_gattc_disc_all_dscs(conn_handle, start_handle, di->end,
                                         gattc_disc_dsc_cb, arg);
            assert(rc == 0);
            break;
        }
        /* no break */
    default:
        /* Disconnect on GATT error */
        ble_gap_terminate(conn_handle, BLE_ERR_CONN_TERM_LOCAL);
        break;
    }

    return 0;
}

static int
gattc_disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                  const struct ble_gatt_svc *svc, void *arg)
{
    struct discov_info *di = arg;
    uint16_t start_handle;
    int rc;

    switch (error->status) {
    case 0:
        if (ble_uuid_u16(&svc->uuid.u) == LNS_UUID) {
            di->start = svc->start_handle;
            di->end = svc->end_handle;
        }
        break;
    case BLE_HS_EDONE:
        /*
         * We need to discover CCC for 'Location and Speed' but for this we need
         * characteristic handle range. Start handle is unknown, so reset it,
         * and the end handle will be not beyond end of found service handle, so
         * keep it.
         */
        start_handle = di->start;
        di->start = 0;

        rc = ble_gattc_disc_all_chrs(conn_handle, start_handle, di->end,
                                     gattc_disc_chr_cb, arg);
        assert(rc == 0);
        break;
    default:
        /* Disconnect on GATT error */
        ble_gap_terminate(conn_handle, BLE_ERR_CONN_TERM_LOCAL);
        break;
    }

    return 0;
}

static int
gap_event_cb(struct ble_gap_event *event, void *arg)
{
    static struct discov_info di;
    uint8_t tx_phy, rx_phy;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            printf("# connected\n");

            ble_gap_read_le_phy(event->connect.conn_handle, &tx_phy, &rx_phy);
            printf("# phy %s/%s\n", phy2str(tx_phy), phy2str(rx_phy));

            di.start = 0;
            di.end = 0;
            ble_gattc_disc_svc_by_uuid(event->connect.conn_handle,
                                       BLE_UUID16_DECLARE(LNS_UUID),
                                       gattc_disc_svc_cb, &di);
        } else {
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        printf("# disconnected\n");
        start_scan();
        break;

    case BLE_GAP_EVENT_EXT_DISC:
        ble_hs_adv_parse(event->ext_disc.data, event->ext_disc.length_data,
                         adv_parse_cb, &event->ext_disc);
        break;

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        printf("# phy %s/%s\n", phy2str(event->phy_updated.tx_phy),
               phy2str(event->phy_updated.rx_phy));
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        process_notify(event->notify_rx.conn_handle, event->notify_rx.om);
        break;
    }

    return 0;
}

static void
start_scan(void)
{
    struct ble_gap_ext_disc_params discp = {};
    int rc;

    discp.itvl = BLE_GAP_SCAN_FAST_WINDOW;
    discp.window = BLE_GAP_SCAN_FAST_WINDOW;
    discp.passive = 1;

    /* XXX: scan uncoded only for now since scanning both phys is broken */
    rc = ble_gap_ext_disc(BLE_OWN_ADDR_RANDOM, 0, 0, 0, 0, 0, &discp, NULL,
                          gap_event_cb, NULL);
    assert(rc == 0);

    printf("# scanning\n");
}

static void
refresh_nrpa(void)
{
    ble_addr_t addr;
    int rc;

    rc = ble_hs_id_gen_rnd(1, &addr);
    assert(rc == 0);

    rc = ble_hs_id_set_rnd(addr.val);
    assert(rc == 0);
}

static void
on_sync_cb(void)
{
    // XXX: this should be called periodically to change NRPA
    refresh_nrpa();

    ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_ANY_MASK,
                                        BLE_GAP_LE_PHY_ANY_MASK);

    start_scan();
}

static void
gpio_ev_cbr(struct os_event *ev)
{
    struct ble_gap_conn_params connp = {};

    ble_gap_disc_cancel();

    connp.scan_itvl = BLE_GAP_SCAN_FAST_WINDOW;
    connp.scan_window = BLE_GAP_SCAN_FAST_WINDOW;
    connp.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN;
    connp.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX;
    connp.latency = 0;
    connp.supervision_timeout = 500; /* 5000 msecs */

    ble_gap_ext_connect(BLE_OWN_ADDR_RANDOM, &peer_addr, BLE_HS_FOREVER,
                        BLE_GAP_LE_PHY_1M_MASK, &connp, NULL, NULL,
                        gap_event_cb, NULL);
}

static void __attribute__((unused))
gpio_irq_handler(void *arg)
{
    gpio_ev.ev_arg = arg;
    os_eventq_put(os_eventq_dflt_get(), &gpio_ev);
}

static void
gpio_setup(void)
{
    hal_gpio_irq_init(BUTTON_1, gpio_irq_handler, (void *)BUTTON_1,
                      HAL_GPIO_TRIG_FALLING, HAL_GPIO_PULL_UP);
    hal_gpio_irq_enable(BUTTON_1);
}

int
main(void)
{
    sysinit();

    ble_hs_cfg.sync_cb = on_sync_cb;

    gpio_setup();
    gpio_ev.ev_cb = gpio_ev_cbr;

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
