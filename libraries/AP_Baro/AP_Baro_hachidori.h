/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

#include "AP_Baro_Backend.h"

class AP_InertialSensor_HACHIDORI;

class AP_Baro_HACHIDORI : public AP_Baro_Backend
{
public:
    AP_Baro_HACHIDORI(AP_Baro &);
    void update();

private:
    void _timer_update();

    uint8_t _baro_instance;

    float _press;
    float _temp;

    uint32_t _last_check_ms;
    bool _updated;
};

#endif // CONFIG_HAL_BOARD_SUBTYPE
