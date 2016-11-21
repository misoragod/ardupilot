/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "AP_Baro_Backend.h"

class AP_Baro_2SMPB02 : public AP_Baro_Backend
{
public:
    // Constructor
    AP_Baro_2SMPB02(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /* AP_Baro public interface: */
    void update();

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:
    virtual ~AP_Baro_2SMPB02(void) {};

    bool _init(void);
    bool _read_raw();
    void _calculate();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

	uint8_t			_instance;
	double			_temp;
	double			_press;

	// Internal calibration registers
	double			k_AA, k_BA, k_CA;
	double			k_AP, k_BP, k_CP;
	double			k_AT, k_BT, k_CT;

	uint32_t		_retry_time;
	int32_t			raw_P;
	int32_t			raw_T;
};
