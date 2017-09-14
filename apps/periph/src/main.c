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
#include "minmea.h"
#include "oled.h"

static struct uart_dev *uart_dev;

static struct gps_info {
    int sat_tracked;
    int sat_total;
    int8_t sat_snr[32];
    int8_t sat_snr_new[32];
    uint32_t sat_mask;

    struct minmea_time time;
    int fix_quality;
    struct minmea_float lon;
    struct minmea_float lat;
    struct minmea_float speed;
    struct minmea_float altitude;
    char altitude_unit;

    struct minmea_float pdop;
    struct minmea_float hdop;
    struct minmea_float vdop;
} gps_info;

static void update_oled_cb(struct os_event *ev);

static struct os_event update_oled_ev = {
    .ev_cb = update_oled_cb,
};

static void
coord_to_dms(struct minmea_float *coord, int *deg, int *min, int *tsec, int *dir)
{
    const int scale = 1000;
    int scaled_coord;

    scaled_coord = minmea_rescale(coord, scale);

    if (scaled_coord < 0) {
        *dir = -1;
        scaled_coord *= -1;
    } else {
        *dir = 1;
    }

    *deg = scaled_coord / (100 * scale);
    scaled_coord -= *deg * (100 * scale);
    *min = scaled_coord / scale;
    scaled_coord -= *min * scale;
    *tsec = 600 * scaled_coord / scale;
}

static int
uc_tx_char(void *arg)
{
    return -1;
}

static void
parse_nmea(const char *buf)
{
    static union {
        struct minmea_sentence_gga gga;
        struct minmea_sentence_gsa gsa;
        struct minmea_sentence_gsv gsv;
        struct minmea_sentence_vtg vtg;
    } frame;
    enum minmea_sentence_id id;
    struct minmea_sat_info *si;
    int i;

    id = minmea_sentence_id(buf, false);

    switch (id) {
    case MINMEA_SENTENCE_GGA:
        if (minmea_parse_gga(&frame.gga, buf)) {
            gps_info.sat_tracked = frame.gga.satellites_tracked;
            gps_info.fix_quality = frame.gga.fix_quality > 0;
            gps_info.lon = frame.gga.longitude;
            gps_info.lat = frame.gga.latitude;
            gps_info.time = frame.gga.time;
            gps_info.altitude = frame.gga.altitude;
            gps_info.altitude_unit = frame.gga.altitude_units;
            os_eventq_put(os_eventq_dflt_get(), &update_oled_ev);
        }
        break;
    case MINMEA_SENTENCE_GSA:
        if (minmea_parse_gsa(&frame.gsa, buf)) {
            gps_info.sat_mask = 0;
            for (i = 0; i < 12; i++) {
                gps_info.sat_mask |= 1 << (frame.gsa.sats[i] - 1);
            }

            gps_info.pdop = frame.gsa.pdop;
            gps_info.hdop = frame.gsa.hdop;
            gps_info.vdop = frame.gsa.vdop;
        }
        break;
    case MINMEA_SENTENCE_GSV:
        if (minmea_parse_gsv(&frame.gsv, buf)) {
            if (frame.gsv.msg_nr == 1) {
                memset(gps_info.sat_snr_new, -1, sizeof(gps_info.sat_snr_new));
            }

            for (i = 0; i < 4; i++) {
                si = &frame.gsv.sats[i];

                if ((si->nr > 0) && (si->nr < 33)) {
                    gps_info.sat_snr_new[si->nr - 1] = si->snr;
                }
            }

            if (frame.gsv.msg_nr == frame.gsv.total_msgs) {
                gps_info.sat_total = frame.gsv.total_sats;
                memcpy(gps_info.sat_snr, gps_info.sat_snr_new,
                       sizeof(gps_info.sat_snr));
                os_eventq_put(os_eventq_dflt_get(), &update_oled_ev);
            }
        }
        break;
    case MINMEA_SENTENCE_VTG:
        if (minmea_parse_vtg(&frame.vtg, buf)) {
            gps_info.speed = frame.vtg.speed_kph;
        }
        break;
    default:
        break;
    }
}

static int
uc_rx_char(void *arg, uint8_t byte)
{
    static char buf[MINMEA_MAX_LENGTH + 1];
    static uint8_t len = 0;

    if (len == 0 && byte != '$') {
        return 0;
    }

    buf[len++] = byte;

    if ((byte != '\n') && (len < MINMEA_MAX_LENGTH)) {
        return 0;
    }

    buf[len] = '\0';

    parse_nmea(buf);

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
draw_signal_bar(int val, bool active)
{
    static uint8_t d[7] = { 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff };
    uint8_t v;

    oled_putd(0x00);

    if (val < 0) {
        v = 0;
    } else if (val == 0) {
        v = 0x80;
    } else if (val > 99) {
        v = 0xff;
    } else {
        val = 7 * val / 100;
        v = d[val];
    }

    if (active) {
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
    struct gps_info gi;
    int deg, min, tsec, dir;
    int pdop, hdop, vdop;
    int speed;
    int altitude;
    int sr;
    int i;

    OS_ENTER_CRITICAL(sr);
    gi = gps_info;
    OS_EXIT_CRITICAL(sr);

    if (gi.fix_quality) {
        speed = minmea_rescale(&gi.speed, 10);
        altitude = minmea_rescale(&gi.altitude, 1);

        coord_to_dms(&gps_info.lat, &deg, &min, &tsec, &dir);
        oled_printfln(0, "%d\xf8%02d'%02d.%d\" %c %4d.%dK", deg, min, tsec / 10,
                      tsec % 10, dir > 0 ? 'N' : 'S', speed / 10, speed % 10);

        coord_to_dms(&gps_info.lon, &deg, &min, &tsec, &dir);
        oled_printfln(1, "%d\xf8%02d'%02d.%d\" %c %6d%c", deg, min, tsec / 10,
                      tsec % 10, dir > 0 ? 'E' : 'W', altitude,
                      gi.altitude_unit);
    } else {
        oled_printfln(0, "--\xf8--'--.-\" -");
        oled_printfln(1, "--\xf8--'--.-\" -");
    }

    oled_printfln(2, "%02d:%02d:%02d UTC", gi.time.hours, gi.time.minutes,
                  gi.time.seconds);

    oled_printfln(5, "sat %d/%d", gi.sat_tracked, gi.sat_total);

    pdop = minmea_rescale(&gps_info.pdop, 10);
    hdop = minmea_rescale(&gps_info.hdop, 10);
    vdop = minmea_rescale(&gps_info.vdop, 10);
    if (pdop < 100) {
        oled_printfln(6, "DOP %d.%d  H%d.%d  V%d.%d", pdop / 10, pdop % 10,
                      hdop / 10, hdop % 10, vdop / 10, vdop % 10);
    } else {
        oled_printfln(6, "DOP -.-  H-.-   V-.-");
    }

    oled_page(7);

    for (i = 0; i < 32; i++) {
        draw_signal_bar(gi.sat_snr[i], gi.sat_mask & (1 << i));
    }
}

int
main(void)
{
    sysinit();

    uart_setup();
    oled_setup();

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
