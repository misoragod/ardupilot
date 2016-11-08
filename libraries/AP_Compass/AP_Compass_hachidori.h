/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_InertialSensor_HACHIDORI;

class AP_Compass_HACHIDORI : public AP_Compass_Backend
{
public:
    AP_Compass_HACHIDORI(Compass &compass);

    bool init(void);
    void read(void);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

private:
    void _timer_update();

    uint8_t _compass_instance;

    Vector3f _accum;
    uint32_t _accum_count;
    uint32_t _last_check_ms;
};

#endif // CONFIG_HAL_BOARD_SUBTYPE
