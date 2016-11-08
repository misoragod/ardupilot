/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AnalogIn_hachidori.h"

#include <algorithm>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include "AnalogIn_hachidori.h"

#include <algorithm>

#include <AP_InertialSensor/AP_InertialSensor_hachidori.h>

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

AnalogSource_HACHIDORI::AnalogSource_HACHIDORI(int16_t pin)
    : _pin(pin)
{
}

void AnalogSource_HACHIDORI::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    _pin = pin;
}

float AnalogSource_HACHIDORI::read_average()
{
    return read_latest();
}

float AnalogSource_HACHIDORI::read_latest()
{
    return _value;
}

float AnalogSource_HACHIDORI::voltage_average()
{
    return _value;
}

float AnalogSource_HACHIDORI::voltage_latest()
{
    return _value;
}

float AnalogSource_HACHIDORI::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL& hal;

AnalogIn_HACHIDORI::AnalogIn_HACHIDORI()
{
}

float AnalogIn_HACHIDORI::board_voltage(void)
{
    return 5.0;
}

AP_HAL::AnalogSource* AnalogIn_HACHIDORI::channel(int16_t pin)
{
    for (uint8_t j = 0; j < HACHIDORI_ADC_MAX_CHANNELS; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new AnalogSource_HACHIDORI(pin);
            return _channels[j];
        }
    }

    hal.console->println("Out of analog channels");
    return NULL;
}

void AnalogIn_HACHIDORI::init()
{
#if 0
    AP_HAL::AnalogSource *source = channel(0);
    source->set_pin(0);
#endif

    hal.scheduler->suspend_timer_procs();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn_HACHIDORI::_timer_update, void));
    hal.scheduler->resume_timer_procs();
}

void AnalogIn_HACHIDORI::_timer_update()
{
    struct HACHIDORI_packet pkt;
    HACHIDORI_PORT::check_port(pkt, false);

    uint32_t now = AP_HAL::millis();
    if (now - _last_check_ms < 100) {
        return;
    }
    _last_check_ms = now;

    if (!HACHIDORI_PORT::updated[HACHIDORI_TOS_BAT]) {
        return;
    }

    uint8_t *d = HACHIDORI_PORT::buf[HACHIDORI_TOS_BAT].data;
    le32_t *fp = reinterpret_cast<le32_t *>(d);
    union { float f; uint32_t u32; } v;

    for (int16_t i = 0; i < HACHIDORI_ADC_MAX_CHANNELS; i++) {
        for (int16_t j=0; j < HACHIDORI_ADC_MAX_CHANNELS; j++) {
            AnalogSource_HACHIDORI *source = _channels[j];

            if (source != NULL && i == source->_pin) {
                v.u32 = le32toh(fp[i]);
                source->_value = v.f;
            }
        }
    }

    HACHIDORI_PORT::updated[HACHIDORI_TOS_BAT] = false;
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
