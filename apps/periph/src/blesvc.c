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

static uint16_t chr_tx_handle;

static uint16_t chr_rx_handle;

static uint16_t conn_handle;

static void chr_notify_cb(struct os_event *ev);

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static struct os_event chr_notify_ev = {
    .ev_cb = chr_notify_cb,
};

static const struct ble_gatt_svc_def svc_def[] = {
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

static void start_advertise(void);

static int
gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
        } else {
            start_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = 0xffff;
        start_advertise();
        break;
    }

    return 0;
}

static void
start_advertise(void)
{
    struct ble_gap_adv_params advp;
    struct ble_hs_adv_fields advf;
    int rc;

    memset(&advf, 0, sizeof advf);
    advf.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    advf.uuids128 = (ble_uuid128_t *) &svc_uuid; /* const */
    advf.num_uuids128 = 1;
    advf.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&advf);
    assert(rc == 0);

    memset(&advf, 0, sizeof advf);
    advf.name = (uint8_t *) ble_svc_gap_device_name();
    advf.name_len = strlen((char *) advf.name);
    advf.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&advf);
    assert(rc == 0);

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

    rc = ble_svc_gap_device_name_set("blegps");
    assert(rc == 0);

    conn_handle = 0xffff;
}

void
blesvc_rx_byte(uint8_t byte)
{
    static uint8_t buf[20];
    static uint8_t len = 0;
    struct os_mbuf *om;

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
