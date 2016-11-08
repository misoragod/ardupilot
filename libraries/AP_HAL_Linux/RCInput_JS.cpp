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

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCInput_JS.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

using namespace Linux;

RCInput_JS::RCInput_JS(const char *path)
{
    _fd = open(path, O_RDONLY|O_NOCTTY|O_NONBLOCK|O_NDELAY);
    if (_fd < 0) {
        AP_HAL::panic("RCInput_JS: Error opening '%s': %s",
		      path, strerror(errno));
    }

    for (int i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++) {
        _chdata[i] = 1500;
    }
}

RCInput_JS::~RCInput_JS()
{
    close(_fd);
}

void RCInput_JS::init()
{

    if (ioctl (_fd, JSIOCGAXES, &_naxes) != 0) {
        AP_HAL::panic("RCInput_JS: can't get number of axes: %s",
		      strerror(errno));
    }

    if (ioctl (_fd, JSIOCGBUTTONS, &_nbuttons) != 0) {
        AP_HAL::panic("RCInput_JS: can't get number of buttons: %s",
		      strerror(errno));
    }
}

static uint16_t pwm(int16_t value)
{
    // -32768 -> 1100micro, 32767 -> 1900micro
    return 1100 + ((1900 - 1100)*((int)value + 0x8000)/0xffff);
}

void RCInput_JS::_timer_tick()
{
    bool update_needed = false;
    static uint32_t count = 0, last_update = 0;

    count++;
    while (1) {
        struct js_event js;
        ssize_t n = ::read(_fd, &js, sizeof(struct js_event));
        if (n != sizeof(struct js_event)) {
            // Update periodically even with no events so to make hartbeat
            if (update_needed || (count - last_update) > 10) {
                _update_periods(_chdata, LINUX_RC_INPUT_NUM_CHANNELS);
                update_needed = false;
                last_update = count;
            }
            return;
        }

        switch (js.type & ~JS_EVENT_INIT) {
        case JS_EVENT_AXIS:
            if (js.number < LINUX_RC_INPUT_NUM_CHANNELS
                && js.number < _naxes) {
                _chdata[js.number] = pwm(js.value);
            }
            update_needed = true;
            break;
	    case JS_EVENT_BUTTON:
            // Ignore buttons ATM.  Perhaps, we can map them to some channels.
            break;
        default:
            break;
        }
    }
}
#endif
