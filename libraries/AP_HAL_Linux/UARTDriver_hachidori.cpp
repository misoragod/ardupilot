/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include <stdlib.h>
#include <cstdio>

#include <AP_InertialSensor/AP_InertialSensor_hachidori.h>

#include "UARTDriver_hachidori.h"

#include <AP_HAL/utility/RingBuffer.h>

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

extern const AP_HAL::HAL& hal;

#define HACHIDORI_DEBUG 0

#include <cassert>

#if HACHIDORI_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("[HACHIDORI]: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

using namespace Linux;

UARTDriver_HACHIDORI::UARTDriver_HACHIDORI(void) :
    UARTDriver(false),
    _last_update_timestamp(0)
{
}

void UARTDriver_HACHIDORI::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (device_path != NULL) {
        UARTDriver::begin(b,rxS,txS);
    }

   if (rxS < 1024) {
       rxS = 2048;
   }
   if (txS < 1024) {
       txS = 2048;
   }

   _readbuf.set_size(rxS);
   _writebuf.set_size(txS);

   _initialised = true;
}

int UARTDriver_HACHIDORI::_write_fd(const uint8_t *buf, uint16_t size)
{
    if (size > HACHIDORI_DATA_LEN - 2) {
        size = HACHIDORI_DATA_LEN - 2;
    }

    struct HACHIDORI_packet pkt;
    pkt.head = HACHIDORI_HAEDER_ID;
    pkt.tos = HACHIDORI_TOS_GPSCMD;
    memcpy(&pkt.data[1], buf, size);
    pkt.data[0] = size;
    HACHIDORI_PORT::send_port(pkt);
    debug ("%d bytes sent\n", size);

    return size;
}

int UARTDriver_HACHIDORI::_read_fd(uint8_t *buf, uint16_t len)
{
    int n;
    if ((n = HACHIDORI_PORT::read_gps (buf, len)) < 0) {
        error ("can't read\n");
        return 0;
    }
    debug ("%d bytes retrieved\n", n);

    return n;
}

void UARTDriver_HACHIDORI::_timer_tick(void)
{
    /* lower the update rate */
    if (AP_HAL::micros() - _last_update_timestamp < 10000) {
        return;
    }

    UARTDriver::_timer_tick();

    _last_update_timestamp = AP_HAL::micros();
}

#endif
