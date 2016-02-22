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

#pragma once

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Linux.h"
#include "RCInput.h"

#define MAX_XBUS_FRAME_LEN 256

namespace Linux {

class RCInput_XBUS : public RCInput
{
public:
    RCInput_XBUS(const char *path);
    ~RCInput_XBUS();

    void init() override;
    void _timer_tick(void) override;

private:
    int _fd;
    int _state;
    int _offset;
    int _length;
    uint8_t _chid;
    uint8_t _chhigh;
    uint8_t _crc;
    uint16_t _chdata[LINUX_RC_INPUT_NUM_CHANNELS];
};

}
