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

#ifndef _BLESVC_H_
#define _BLESVC_H_

#ifdef __cplusplus
extern "C" {
#endif

void blesvc_setup(void);

uint16_t blesvc_get_conn_handle(void);

uint8_t blesvc_get_conn_phy(void);

void blesvc_rx_byte(uint8_t byte);

void blesvc_notify(void);

#ifdef __cplusplus
}
#endif

#endif /* _BLESVC_H */
