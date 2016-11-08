/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include <AP_InertialSensor/AP_InertialSensor_hachidori.h>

#include "RCOutput_hachidori.h"

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCOutput_HACHIDORI::init()
{
    HACHIDORI_PORT::open_port();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&RCOutput_HACHIDORI::_timer_update, void));
}

void RCOutput_HACHIDORI::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    // no support for changing frequency yet
}

uint16_t RCOutput_HACHIDORI::get_freq(uint8_t ch)
{
    // return fixed fake value
    return 200;
}

void RCOutput_HACHIDORI::enable_ch(uint8_t ch)
{

}

void RCOutput_HACHIDORI::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

void RCOutput_HACHIDORI::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= _channel_count) {
        return;
    }
    _period[ch] = period_us;
    if (!_corked) {
        _need_write = true;
    }
}

void RCOutput_HACHIDORI::cork(void)
{
    _corked = true;
}

void RCOutput_HACHIDORI::push(void)
{
    _corked = false;
    _need_write = true;
}

uint16_t RCOutput_HACHIDORI::read(uint8_t ch)
{
    if (ch >= _channel_count) {
        return 0;
    }
    return _period[ch];
}

void RCOutput_HACHIDORI::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void RCOutput_HACHIDORI::_timer_update(void)
{
    int fd = HACHIDORI_PORT::get_port();
    if (!_need_write || fd < 0) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (now - _last_check_ms < 5) {
        return;
    }
    _last_check_ms = now;

    struct HACHIDORI_packet pkt;
    pkt.head = HACHIDORI_HAEDER_ID;
    pkt.tos = HACHIDORI_TOS_PWM;
    uint8_t *d = pkt.data;
    for (int i = 0; i < _channel_count; i++) {
        d[2*i] = _period[i] >> 8;
        d[2*i+1] = _period[i] & 0xFF;
    }

    HACHIDORI_PORT::send_port(pkt);
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
