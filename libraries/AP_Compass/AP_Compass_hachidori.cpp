/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include <unistd.h>

#include "AP_Compass_hachidori.h"
#include <AP_InertialSensor/AP_InertialSensor_hachidori.h>

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Compass_HACHIDORI::AP_Compass_HACHIDORI(Compass &compass)
    : AP_Compass_Backend(compass)
{
        _accum.zero();
        _accum_count = 0;
}

// detect the sensor
AP_Compass_Backend *AP_Compass_HACHIDORI::detect(Compass &compass)
{
    HACHIDORI_PORT::open_port();
    AP_Compass_HACHIDORI *sensor = new AP_Compass_HACHIDORI(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_Compass_HACHIDORI::init(void)
{
    _compass_instance = register_compass();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_HACHIDORI::_timer_update, void));
    // give time for at least one sample
    hal.scheduler->delay(100);
    return true;
}

void AP_Compass_HACHIDORI::read(void)
{
    if (_accum_count > 0) {
        hal.scheduler->suspend_timer_procs();
        publish_filtered_field(_accum/_accum_count, _compass_instance);
        _accum.zero();
        _accum_count = 0;
        hal.scheduler->resume_timer_procs();
    }
}

void AP_Compass_HACHIDORI::_timer_update(void)
{
    struct HACHIDORI_packet pkt;
    HACHIDORI_PORT::check_port(pkt, false);

    uint32_t now = AP_HAL::millis();
    if (now - _last_check_ms < 10) {
        return;
    }
    _last_check_ms = now;

    if (HACHIDORI_PORT::updated[HACHIDORI_TOS_MAG]) {
        // deserialize packet bytes to floats
        uint8_t *d = HACHIDORI_PORT::buf[HACHIDORI_TOS_MAG].data;
        le32_t *fp = reinterpret_cast<le32_t *>(d);
        union { float f; uint32_t u32; } x, y, z;
        x.u32 = le32toh(fp[0]);
        y.u32 = le32toh(fp[1]);
        z.u32 = le32toh(fp[2]);

        // get raw_field - sensor frame, uncorrected
        Vector3f raw_field(x.f, y.f, z.f);

        HACHIDORI_PORT::updated[HACHIDORI_TOS_MAG] = false;

        // rotate raw_field from sensor frame to body frame
        rotate_field(raw_field, _compass_instance);

        // publish raw_field (uncorrected point sample) for calibration use
        publish_raw_field(raw_field, _compass_instance);

        // correct raw_field for known errors
        correct_field(raw_field, _compass_instance);

        // accumulate into averaging filter
        _accum += raw_field;
        _accum_count++;
        if (_accum_count == 10) {
            _accum *= 0.5;
            _accum_count = 5;
        }
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
