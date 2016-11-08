/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_ADC/AP_ADC.h>

#include "AP_HAL_Linux.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#define HACHIDORI_ADC_MAX_CHANNELS 2

class AnalogSource_HACHIDORI: public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_HACHIDORI;
    AnalogSource_HACHIDORI(int16_t pin);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p) { }
    void set_settle_time(uint16_t settle_time_ms) { }
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();
private:
    int16_t _pin;
    float _value;
};

class AnalogIn_HACHIDORI: public AP_HAL::AnalogIn {
public:
    AnalogIn_HACHIDORI();

    void init();
    AP_HAL::AnalogSource* channel(int16_t n);

    // Board voltage is not available
    float board_voltage(void);

protected:
    void _timer_update();

    AnalogSource_HACHIDORI *_channels[HACHIDORI_ADC_MAX_CHANNELS];

    uint32_t _last_check_ms;
};

#endif // CONFIG_HAL_BOARD_SUBTYPE
