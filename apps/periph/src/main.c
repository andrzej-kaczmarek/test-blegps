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

#include "sysinit/sysinit.h"
#include "bsp/bsp.h"
#include "os/os.h"
#include "uart/uart.h"
#include "gps.h"
#include "minmea.h"
#include "oled.h"
#include "blesvc.h"

static struct uart_dev *uart_dev;

static void update_oled_cb(struct os_event *ev);

static struct os_event update_oled_ev = {
    .ev_cb = update_oled_cb,
};

static void
coord_to_dms(int coord, int *deg, int *min, int *tsec, int *dir)
{
    if (coord < 0) {
        *dir = -1;
        coord *= -1;
    } else {
        *dir = 1;
    }

    /*
     * calculations below are base on an assumption that coord is a fixed point
     * number stored as: DDMM.MMM
     */

    *deg = coord / 100000;
    coord -= *deg * 100000;
    *min = coord / 1000;
    coord -= *min * 1000;
    *tsec = 600 * coord / 1000;
}

static int
uc_tx_char(void *arg)
{
    return -1;
}

static int
uc_rx_char(void *arg, uint8_t byte)
{
    static char buf[MINMEA_MAX_LENGTH + 1];
    static uint8_t len = 0;

    blesvc_rx_byte(byte);

    if (len == 0 && byte != '$') {
        return 0;
    }

    buf[len++] = byte;

    if ((byte != '\n') && (len < MINMEA_MAX_LENGTH)) {
        return 0;
    }

    buf[len] = '\0';

    if (gps_parse_nmea(buf)) {
        os_eventq_put(os_eventq_dflt_get(), &update_oled_ev);
    }

    len = 0;

    return 0;
}

static void
uart_setup(void)
{
    struct uart_conf uc = {
        .uc_speed = 9600,
        .uc_databits = 8,
        .uc_stopbits = 1,
        .uc_parity = UART_PARITY_NONE,
        .uc_flow_ctl = UART_FLOW_CTL_NONE,
        .uc_tx_char = uc_tx_char,
        .uc_rx_char = uc_rx_char,
    };

    if (!uart_dev) {
        uart_dev = (struct uart_dev *) os_dev_open("uart0", OS_TIMEOUT_NEVER, &uc);
    }
}

static inline void
draw_signal_bar(const struct gps_sat_info *gsi, int max)
{
    static uint8_t d[7] = { 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff };
    uint8_t v;

    oled_putd(0x00);

    if (!gsi->present) {
        v = 0;
    } else if (gsi->snr == 0) {
        /* draw at least single bar for present satellites */
        v = 0x80;
    } else if (gsi->snr > 99) {
        /* draw full bar for anything out of range (if ever happened) */
        v = 0xff;
    } else {
        v = d[7 * gsi->snr / (max + 1)];
    }

    if (gsi->used) {
        oled_putd(v);
        oled_putd(v);
        oled_putd(v);
    } else {
        oled_putd(0x00);
        oled_putd(v);
        oled_putd(0x00);
    }
}

static void
update_oled_cb(struct os_event *ev)
{
    int deg, min, tsec, dir;
    int i;

    gps_info_update();

    if (gps_info.fix_quality) {
        coord_to_dms(gps_info.lat, &deg, &min, &tsec, &dir);
        oled_printfln(0, "%d\xf8%02d'%02d.%d\" %c %4d.%dk",
                      deg, min, tsec / 10, tsec % 10, dir > 0 ? 'N' : 'S',
                      (int) gps_info.speed / 100,
                      (int) gps_info.speed / 10 % 10);

        coord_to_dms(gps_info.lon, &deg, &min, &tsec, &dir);
        oled_printfln(1, "%d\xf8%02d'%02d.%d\" %c %6dm",
                      deg, min, tsec / 10, tsec % 10, dir > 0 ? 'E' : 'W',
                      (int) gps_info.altitude / 100);
    } else {
        oled_printfln(0, "--\xf8--'--.-\" -");
        oled_printfln(1, "--\xf8--'--.-\" -");
    }

    oled_printfln(2, "%02d:%02d:%02d UTC", gps_info.time.hours,
                  gps_info.time.minutes, gps_info.time.seconds);

    oled_printfln(5, "sat %d/%d", gps_info.sat_tracked, gps_info.sat_total);

    if (gps_info.pdop < 100) {
        oled_printfln(6, "DOP %d.%d  H%d.%d  V%d.%d",
                      gps_info.pdop / 10, gps_info.pdop % 10,
                      gps_info.hdop / 10, gps_info.hdop % 10,
                      gps_info.vdop / 10, gps_info.vdop % 10);
    } else {
        oled_printfln(6, "DOP -.-  H-.-   V-.-");
    }

    oled_page(7);

    for (i = 0; i < GPS_INFO_SAT_COUNT; i++) {
        draw_signal_bar(&gps_info.sat[i], gps_info.sat_snr_max);
    }
}

int
main(void)
{
    sysinit();

    uart_setup();
    oled_setup();
    blesvc_setup();

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
