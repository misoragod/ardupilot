/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_Linux.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

class AP_InertialSensor_HACHIDORI;

namespace Linux {

class RCOutput_HACHIDORI : public AP_HAL::RCOutput {
public:
    void init();
    void set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void enable_ch(uint8_t ch);
    void disable_ch(uint8_t ch);
    void write(uint8_t ch, uint16_t period_us);
    void cork();
    void push();
    uint16_t read(uint8_t ch);
    void read(uint16_t *period_us, uint8_t len);

private:
    void _timer_update(void);

    static const uint8_t _channel_count = 16;
    uint16_t _period[_channel_count];
    uint32_t _last_check_ms;
    volatile bool _need_write;
    bool _corked;
};

}

#endif // CONFIG_HAL_BOARD_SUBTYPE
