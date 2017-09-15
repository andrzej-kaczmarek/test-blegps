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

#ifndef _GPS_H_
#define _GPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define GPS_INFO_SAT_COUNT  32

struct gps_sat_info {
    bool present : 1;
    bool used : 1;
    uint8_t snr;
};

struct gps_info {
    uint8_t fix_quality;
    uint8_t sat_tracked;
    uint8_t sat_total;
    struct gps_sat_info sat[GPS_INFO_SAT_COUNT];
    uint8_t sat_snr_max;

    int16_t pdop;
    int16_t hdop;
    int16_t vdop;

    int32_t lat;
    int32_t lon;

    int32_t speed;
    int32_t altitude;

    struct {
        uint8_t hours;
        uint8_t minutes;
        uint8_t seconds;
    } time;
};

extern struct gps_info gps_info;

bool gps_parse_nmea(const char *nmea_str);

void gps_info_update(void);

#ifdef __cplusplus
}
#endif

#endif /* _GPS_H */
