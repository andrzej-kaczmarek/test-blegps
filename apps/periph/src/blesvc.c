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

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "os/endian.h"
#include "console/console.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "gps.h"

static const char gap_name[] = "blegps";

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

#define LNS_PQ_FLAG_NUM_SAT_TRACKED             0x0001
#define LNS_PQ_FLAG_NUM_SAT_TOTAL               0x0002
#define LNS_PQ_FLAG_TIME_TO_FIRST_FIX           0x0004
#define LNS_PQ_FLAG_EHPE                        0x0008
#define LNS_PQ_FLAG_EVPE                        0x0010
#define LNS_PQ_FLAG_HDOP                        0x0020
#define LNS_PQ_FLAG_VDOP                        0x0040
#define LNS_PQ_FLAG_POS_NO                      0x0000
#define LNS_PQ_FLAG_POS_OK                      0x0080
#define LNS_PQ_FLAG_POS_EST                     0x0100
#define LNS_PQ_FLAG_POS_LAST_KNOWN              0x0180

/* {6E400001-B5A3-F393-E0A9-E50E24DCCA9E} */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* {6E400002-B5A3-F393-E0A9-E50E24DCCA9E} */
static const ble_uuid128_t chr_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);


/* {6E400003-B5A3-F393-E0A9-E50E24DCCA9E} */
static const ble_uuid128_t chr_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

#define LNS_UUID                        0x1819
#define LNS_LN_FEATURE_UUID             0x2A6A
#define LNS_LOCATION_AND_SPEED_UUID     0x2A67
#define LNS_POSITION_QUALITY_UUID       0x2A69

static uint16_t chr_tx_handle;

static uint16_t chr_rx_handle;

static bool nuart_tx_notify;

static uint16_t lns_las_handle;

static bool lns_las_notify;

static uint16_t conn_handle = 0xFFFF;

static uint8_t conn_phy;

static void chr_notify_cb(struct os_event *ev);

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static int lns_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static struct os_event chr_notify_ev = {
    .ev_cb = chr_notify_cb,
};

static const struct ble_gatt_svc_def svc_def[] = {
    {
        /* Service: LNS */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(LNS_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) { {
            /* Characteristic: LN Feature */
            .uuid = BLE_UUID16_DECLARE(LNS_LN_FEATURE_UUID),
            .access_cb = lns_access_cb,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            /* Characteristic: Location and Speed */
            .uuid = BLE_UUID16_DECLARE(LNS_LOCATION_AND_SPEED_UUID),
            .access_cb = lns_access_cb,
            .val_handle = &lns_las_handle,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, {
            /* Characteristic: Position Quality */
            .uuid = BLE_UUID16_DECLARE(LNS_POSITION_QUALITY_UUID),
            .access_cb = lns_access_cb,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            0, /* No more characteristics in this service */
        } },
    },

    {
        /* Service: uart */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &chr_tx_uuid.u,
            .val_handle = &chr_tx_handle,
            .access_cb = chr_access_cb,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, {
            /* Characteristic: Write */
            .uuid = &chr_rx_uuid.u,
            .access_cb = chr_access_cb,
            .val_handle = &chr_rx_handle,
            .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        }, {
            0, /* No more characteristics in this service */
        } },
    },

    {
        0, /* No more services */
    },
};

static void
chr_notify_cb(struct os_event *ev)
{
    struct os_mbuf *om;
    int sr;

    om = ev->ev_arg;

    assert(om);

    ble_gattc_notify_custom(conn_handle, chr_tx_handle, om);

    OS_ENTER_CRITICAL(sr);
    ev->ev_arg = NULL;
    OS_EXIT_CRITICAL(sr);
}

static int
chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* We don't really care about this so can accept anything. */

    return 0;
}

static int
lns_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* speed, location, elevation, sat tracked/total, hdop, vdop, pos status */
    static const uint32_t feat = htole32(0x00118c0d);
    struct {
        uint16_t flags;
        uint8_t sat_tracked;
        uint8_t sat_total;
        uint8_t hdop;
        uint8_t vdop;
    } __attribute__((packed)) posq;
    uint16_t uuid;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    switch (uuid) {
    case LNS_LN_FEATURE_UUID:
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
        os_mbuf_append(ctxt->om, &feat, sizeof(feat));
        break;
    case LNS_POSITION_QUALITY_UUID:
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
        if (gps_info.fix_quality > 0) {
            posq.flags = htole16(LNS_PQ_FLAG_NUM_SAT_TRACKED |
                                 LNS_PQ_FLAG_NUM_SAT_TOTAL |
                                 LNS_PQ_FLAG_HDOP | LNS_PQ_FLAG_VDOP |
                                 LNS_PQ_FLAG_POS_OK);

            posq.sat_tracked = gps_info.sat_tracked;
            posq.sat_total = gps_info.sat_total;
            posq.hdop = gps_info.hdop / 2;
            posq.vdop = gps_info.vdop / 2;

            os_mbuf_append(ctxt->om, &posq, sizeof(posq));
        } else {
            posq.flags = htole16(LNS_PQ_FLAG_POS_NO);
            os_mbuf_append(ctxt->om, &posq, sizeof(posq.flags));
        }
        break;
    }

    return 0;
}

static void start_advertise(void);

static int
gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            /* Start with 1M since we use legacy advertising */
            conn_phy = BLE_GAP_LE_PHY_1M;
        } else {
            start_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = 0xffff;
        start_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == lns_las_handle) {
            lns_las_notify = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == chr_tx_handle) {
            nuart_tx_notify = event->subscribe.cur_notify;
        }
        break;

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        if ((event->phy_updated.status == 0) &&
                            (event->phy_updated.conn_handle == conn_handle)) {
            conn_phy = event->phy_updated.tx_phy;
        }
        break;
    }

    return 0;
}

static inline int32_t
coord_to_loc(int32_t coord)
{
    int d, m;

    d = coord / 100000;
    m = coord % 100000;

    return d * 10000000 + m * 1000 / 6;
}

static void
put_ad(uint8_t ad_type, uint8_t ad_len, const void *ad, uint8_t *buf,
       uint8_t *len)
{
    buf[(*len)++] = ad_len + 1;
    buf[(*len)++] = ad_type;

    memcpy(&buf[*len], ad, ad_len);

    *len += ad_len;
}

static void
update_ad(void)
{
    static bool ad_init = false;
    static uint8_t ad[BLE_HS_ADV_MAX_SZ];
    static uint8_t ad_len = 0;
    static uint8_t ad_flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    static uint16_t ad_uuid = htole16(LNS_UUID);
    struct {
        uint16_t uuid;
        int32_t lat;
        int32_t lon;
    } __attribute__((packed)) ad_svcdata;
    uint8_t len;

    if (!ad_init) {
        put_ad(BLE_HS_ADV_TYPE_FLAGS, 1, &ad_flags, ad, &ad_len);
        put_ad(BLE_HS_ADV_TYPE_COMP_NAME, sizeof(gap_name), gap_name,
               ad, &ad_len);
        put_ad(BLE_HS_ADV_TYPE_INCOMP_UUIDS16, sizeof(ad_uuid), &ad_uuid,
               ad, &ad_len);
        ad_init = true;
    }

    len = ad_len;

    if (gps_info.fix_quality) {
        /* This is not defined by LNS spec! */
        ad_svcdata.uuid = LNS_UUID;
        ad_svcdata.lat = htole32(coord_to_loc(gps_info.lat));
        ad_svcdata.lon = htole32(coord_to_loc(gps_info.lon));

        put_ad(BLE_HS_ADV_TYPE_SVC_DATA_UUID16, sizeof(ad_svcdata), &ad_svcdata,
               ad, &len);
    }

    ble_gap_adv_set_data(ad, len);
}

static void
start_advertise(void)
{
    struct ble_gap_adv_params advp;
    int rc;

    update_ad();

    memset(&advp, 0, sizeof advp);
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &advp, gap_event_cb, NULL);
    assert(rc == 0);
}

static void
on_sync_cb(void)
{
    start_advertise();
}

void
blesvc_setup(void)
{
    int rc;

    log_register("ble_hs", &ble_hs_log, &log_console_handler, NULL, LOG_SYSLEVEL);
    ble_hs_cfg.sync_cb = on_sync_cb;

    rc = ble_gatts_count_cfg(svc_def);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(svc_def);
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set(gap_name);
    assert(rc == 0);

    conn_handle = 0xffff;
}

uint16_t
blesvc_get_conn_handle(void)
{
    return conn_handle;
}

uint8_t blesvc_get_conn_phy(void)
{
    return conn_phy;
}

void
blesvc_rx_byte(uint8_t byte)
{
    static uint8_t buf[20];
    static uint8_t len = 0;
    struct os_mbuf *om;

    if (!nuart_tx_notify) {
        goto done;
    }

    buf[len++] = byte;

    if (len < sizeof(buf)) {
        return;
    }

    if (chr_notify_ev.ev_arg) {
        goto done;
    }

    om = ble_hs_mbuf_from_flat(buf, len);
    if (!om) {
        goto done;
    }

    chr_notify_ev.ev_arg = om;

    os_eventq_put(os_eventq_dflt_get(), &chr_notify_ev);

done:
    len = 0;
}

void blesvc_notify(void)
{
    struct {
        uint16_t flags;
        uint16_t speed;
        int32_t lat;
        int32_t lon;
        uint8_t elevation[3];
    } __attribute__((packed)) pkt;
    int32_t elevation;
    struct os_mbuf *om;

    update_ad();

    if (!lns_las_notify) {
        return;
    }

    if (gps_info.fix_quality > 0) {
        pkt.flags = htole16(LNS_LAS_FLAG_SPEED | LNS_LAS_FLAG_LOCATION |
                            LNS_LAS_FLAG_ELEVATION | LNS_LAS_FLAG_POS_OK |
                            LNS_LAS_FLAG_SPD_2D |
                            LNS_LAS_FLAG_ELEVATION_SRC_GPS);

        pkt.speed = htole16(gps_info.speed * 10 / 36);
        pkt.lat = htole32(coord_to_loc(gps_info.lat));
        pkt.lon = htole32(coord_to_loc(gps_info.lon));

        elevation = htole32(gps_info.altitude);
        memcpy(pkt.elevation, &elevation, 3);

        om = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt));
    } else {
        pkt.flags = htole16(LNS_LAS_FLAG_POS_NO);
        om = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt.flags));
    }

    ble_gattc_notify_custom(conn_handle, lns_las_handle, om);
}
