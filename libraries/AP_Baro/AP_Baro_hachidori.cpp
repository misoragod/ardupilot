/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include <unistd.h>

#include "AP_Baro_hachidori.h"
#include <AP_InertialSensor/AP_InertialSensor_hachidori.h>

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Baro_HACHIDORI::AP_Baro_HACHIDORI(AP_Baro &baro)
    : AP_Baro_Backend(baro)
    , _updated(false)
{
    HACHIDORI_PORT::open_port();
    _baro_instance = _frontend.register_sensor();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Baro_HACHIDORI::_timer_update, void));
}

void AP_Baro_HACHIDORI::_timer_update(void)
{
    struct HACHIDORI_packet pkt;
    HACHIDORI_PORT::check_port(pkt, false);

    uint32_t now = AP_HAL::millis();
    if (now - _last_check_ms < 10) {
        return;
    }
    _last_check_ms = now;


    if (HACHIDORI_PORT::updated[HACHIDORI_TOS_BARO]) {
        // deserialize packet bytes to floats
        uint8_t *d = HACHIDORI_PORT::buf[HACHIDORI_TOS_BARO].data;
        le32_t *fp = reinterpret_cast<le32_t *>(d);
        union { float f; uint32_t u32; } p, t;
        p.u32 = le32toh(fp[0]);
        t.u32 = le32toh(fp[1]);

        // get float values
        _press = p.f;
        _temp = t.f;
        _updated = true;
        HACHIDORI_PORT::updated[HACHIDORI_TOS_BARO] = false;
    }
}

void AP_Baro_HACHIDORI::update(void)
{
    if (_updated) {
        hal.scheduler->suspend_timer_procs();

        float temperature = _temp;
        float pressure = _press;

        _copy_to_frontend(_baro_instance, pressure, temperature);
        _updated = false;
        hal.scheduler->resume_timer_procs();
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
