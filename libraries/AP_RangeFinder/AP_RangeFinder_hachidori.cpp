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
#include <AP_HAL/utility/sparse-endian.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include "AP_RangeFinder_hachidori.h"
#include <AP_InertialSensor/AP_InertialSensor_hachidori.h>

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_HACHIDORI::AP_RangeFinder_HACHIDORI(RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_state)
{
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_RangeFinder_HACHIDORI::_timer_update, void));
}

/*
   detect if a rangefinder packet is gotten.
*/
bool AP_RangeFinder_HACHIDORI::detect(void)
{
    HACHIDORI_PORT::open_port();
    // give time for at least one sample
    hal.scheduler->delay(100);

    if (!HACHIDORI_PORT::updated[HACHIDORI_TOS_RANGE]) {
        return false;
    }

    return true;
}

/*
  timer rated at 20Hz
*/
void AP_RangeFinder_HACHIDORI::_timer_update(void)
{
    struct HACHIDORI_packet pkt;
    HACHIDORI_PORT::check_port(pkt, false);

    uint32_t now = AP_HAL::millis();
    if (now - _last_check_ms < 50) {
        return;
    }
    _last_check_ms = now;

    if (HACHIDORI_PORT::updated[HACHIDORI_TOS_RANGE]) {
        // deserialize packet bytes to floats
        uint8_t *d = HACHIDORI_PORT::buf[HACHIDORI_TOS_RANGE].data;
        le32_t *fp = reinterpret_cast<le32_t *>(d);
        union { float f; uint32_t u32; } x;
        x.u32 = le32toh(fp[0]);
        HACHIDORI_PORT::updated[HACHIDORI_TOS_RANGE] = false;

        // a negative result means that messurement fails this time
        if (x.f >= 0) {
            if (_sem->take(0)) {
                _distance = (uint16_t)x.f;
                _new_distance = true;
                _sem->give();
            }
        }
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_HACHIDORI::update(void)
{
    if (_sem->take_nonblocking()) {
        if (_new_distance) {
            state.distance_cm = _distance;
            _new_distance = false;
            update_status();
        } else {
            // This doesn't work yet because of its very slow range finder.
            //set_status(RangeFinder::RangeFinder_NoData);
        }
        _sem->give();
    }
}
#endif // CONFIG_HAL_BOARD_SUBTYPE
