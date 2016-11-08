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

#include "AP_HAL_Linux.h"

#include "UARTDriver.h"

namespace Linux {

class UARTDriver_HACHIDORI : public UARTDriver {
public:
    UARTDriver_HACHIDORI(void);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void _timer_tick(void);

protected:
    int _write_fd(const uint8_t *buf, uint16_t n);
    int _read_fd(uint8_t *buf, uint16_t n);

private:
    uint32_t _last_update_timestamp;
};

}
