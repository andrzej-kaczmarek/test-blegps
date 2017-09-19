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
#include <stdint.h>
#include <os/os.h>
#include "minmea.h"
#include "gps.h"

static struct nmea_data {
    int sat_tracked;
    int sat_total;
    int8_t sat_snr[GPS_INFO_SAT_COUNT];
    int8_t sat_snr_new[GPS_INFO_SAT_COUNT];
    uint32_t sat_mask;

    struct minmea_time time;
    int fix_quality;
    struct minmea_float lon;
    struct minmea_float lat;
    struct minmea_float speed;
    struct minmea_float altitude;

    struct minmea_float pdop;
    struct minmea_float hdop;
    struct minmea_float vdop;
} nmea_data;

struct gps_info gps_info;

void
gps_parse_nmea(const char *nmea_str)
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

    id = minmea_sentence_id(nmea_str, false);

    switch (id) {
    case MINMEA_SENTENCE_GGA:
        if (minmea_parse_gga(&frame.gga, nmea_str)) {
            nmea_data.sat_tracked = frame.gga.satellites_tracked;
            nmea_data.fix_quality = frame.gga.fix_quality > 0;
            nmea_data.lon = frame.gga.longitude;
            nmea_data.lat = frame.gga.latitude;
            nmea_data.time = frame.gga.time;
            nmea_data.altitude = frame.gga.altitude;
            assert(frame.gga.altitude_units == 'M');
        }
        break;
    case MINMEA_SENTENCE_GSA:
        if (minmea_parse_gsa(&frame.gsa, nmea_str)) {
            nmea_data.sat_mask = 0;
            for (i = 0; i < 12; i++) {
                if (frame.gsa.sats[i] > 0) {
                    nmea_data.sat_mask |= 1 << (frame.gsa.sats[i] - 1);
                }
            }

            nmea_data.pdop = frame.gsa.pdop;
            nmea_data.hdop = frame.gsa.hdop;
            nmea_data.vdop = frame.gsa.vdop;
        }
        break;
    case MINMEA_SENTENCE_GSV:
        if (minmea_parse_gsv(&frame.gsv, nmea_str)) {
            if (frame.gsv.msg_nr == 1) {
                memset(nmea_data.sat_snr_new, -1, sizeof(nmea_data.sat_snr_new));
            }

            for (i = 0; i < 4; i++) {
                si = &frame.gsv.sats[i];

                if ((si->nr > 0) && (si->nr <= GPS_INFO_SAT_COUNT)) {
                    nmea_data.sat_snr_new[si->nr - 1] = si->snr;
                }
            }

            if (frame.gsv.msg_nr == frame.gsv.total_msgs) {
                nmea_data.sat_total = frame.gsv.total_sats;
                memcpy(nmea_data.sat_snr, nmea_data.sat_snr_new,
                       sizeof(nmea_data.sat_snr));
            }
        }
        break;
    case MINMEA_SENTENCE_VTG:
        if (minmea_parse_vtg(&frame.vtg, nmea_str)) {
            nmea_data.speed = frame.vtg.speed_kph;
        }
        break;
    default:
        break;
    }
}

void
gps_info_update(void)
{
    static struct nmea_data nd;
    struct gps_sat_info *gsi;
    int sr;
    int i;

    OS_ENTER_CRITICAL(sr);
    nd = nmea_data;
    OS_EXIT_CRITICAL(sr);

    gps_info.fix_quality = nd.fix_quality;
    gps_info.sat_tracked = nd.sat_tracked;
    gps_info.sat_total = nd.sat_total;
    gps_info.sat_snr_max = 0;

    for (i = 0; i < GPS_INFO_SAT_COUNT; i++) {
        gsi = &gps_info.sat[i];

        gsi->used = nd.sat_mask & (1 << i);
        gsi->present = nd.sat_snr[i] >= 0;
        gsi->snr = nd.sat_snr[i];

        if (gsi->present && (gsi->snr > gps_info.sat_snr_max)) {
            gps_info.sat_snr_max = gsi->snr;
        }
    }

    gps_info.pdop = minmea_rescale(&nd.pdop, 10);
    gps_info.hdop = minmea_rescale(&nd.hdop, 10);
    gps_info.vdop = minmea_rescale(&nd.vdop, 10);

    gps_info.lat = minmea_rescale(&nd.lat, 1000);
    gps_info.lon = minmea_rescale(&nd.lon, 1000);

    gps_info.speed = minmea_rescale(&nd.speed, 100);
    gps_info.altitude = minmea_rescale(&nd.altitude, 100);

    gps_info.time.hours = nd.time.hours;
    gps_info.time.minutes = nd.time.minutes;
    gps_info.time.seconds = nd.time.seconds;
}
