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

    /* XXX: output data somewhere */
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

int
main(void)
{
    sysinit();

    uart_setup();

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
