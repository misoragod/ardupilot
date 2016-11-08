/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_InertialSensor_HACHIDORI;

class AP_RangeFinder_HACHIDORI : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_HACHIDORI(RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(void);

    // update state
    void update(void);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    void _timer_update(void);

    uint32_t _last_check_ms;
    uint16_t _distance;
    bool _new_distance;
};
