/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
   OMRON 2SMPB02 barometer driver.
*/
#include "AP_Baro_2SMPB02.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#include "AP_Baro.h"

extern const AP_HAL::HAL& hal;

#define OMRON_2SMPB02_REG_CALIB0 0xa0
#define OMRON_2SMPB02_CALIB_SIZE (0xb5-0xa0)
#define OMRON_2SMPB02_REG_CTRL 0xf4
#define OMRON_2SMPB02_REG_STAT 0xf3
#define OMRON_2SMPB02_REG_IIR 0xf1
#define OMRON_2SMPB02_REG_RAW 0xf7
#define OMRON_2SMPB02_RAW_SIZE (0xfd-0xf7)

#define DATA_NOT_READY (1 << 3)

// Pressure oversampling x32
#define P_OVERSAMPLING (5 << 2)
// Temperature oversampling x4
#define T_OVERSAMPLING (3 << 5)

/*
  constructor
 */
AP_Baro_2SMPB02::AP_Baro_2SMPB02(AP_Baro &baro,  AP_HAL::OwnPtr<AP_HAL::Device> dev) :
    AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
}

AP_Baro_Backend *AP_Baro_2SMPB02::probe(AP_Baro &baro,
                                        AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_Baro_2SMPB02 *sensor = new AP_Baro_2SMPB02(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_2SMPB02::_init()
{
    uint8_t buff[OMRON_2SMPB02_CALIB_SIZE];

    if (!_dev | !_dev->get_semaphore()->take(0)) {
        return false;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // filter = 5
    _dev->write_register(OMRON_2SMPB02_REG_IIR, 5);

    // We read the calibration data registers
    if (!_dev->read_registers(OMRON_2SMPB02_REG_CALIB0, buff,
                              OMRON_2SMPB02_CALIB_SIZE)) {
        _dev->get_semaphore()->give();
        return false;
    }

    int32_t cp, bp, ap, ct, bt, at, ca, ba, aa;
    cp = ((int8_t)buff[0] << 16) | (buff[1] << 8) | buff[2];
    bp = ((int8_t)buff[3] << 8) | buff[4];
    ap = ((int8_t)buff[5] << 8) | buff[6];

    ct = ((int8_t)buff[7] << 8) | buff[8];
    bt = ((int8_t)buff[9] << 8) | buff[10];
    at = ((int8_t)buff[11] << 8) | buff[12];

    ca = ((int8_t)buff[13] << 16) | (buff[14] << 8) | buff[15];
    ba = ((int8_t)buff[17] << 8) | buff[18];
    aa = ((int8_t)buff[19] << 8) | buff[20];

    k_AA =  0.0E+00 + 4.2E-04 * aa / 32767;
    k_BA = -1.6E+02 + 8.0E+00 * ba / 32767;
    k_CA = (double) ca;
    k_AP =  0.0E+00 + 3.0E-05 * ap / 32767;
    k_BP =  3.0E+01 + 1.0E+01 * bp / 32767;
    k_CP = (double) cp;
    k_AT =  0.0E+00 + 8.0E-11 * at / 32767;
    k_BT = -6.6E-06 + 1.6E-06 * bt / 32767;
    k_CT =  4.0E-02 + 8.5E-03 * ct / 32767;

    _instance = _frontend.register_sensor();

    //Send a command to readout result of conversion
    // OVERSAMPLING, trigger conversion (mode=1)
    _dev->write_register(OMRON_2SMPB02_REG_CTRL,
                         T_OVERSAMPLING | T_OVERSAMPLING | 1);

    _dev->get_semaphore()->give();

    return true;
}

// Read the sensor if ready and calculate tempreture and pressure.
// transfer data to the frontend
void AP_Baro_2SMPB02::update(void)
{
    if (!_sem->take_nonblocking()) {
        return;
    }

    // 2SMPB02 has a data not ready flag instead of EOC output.
    uint8_t stat;
    if (!_dev->read_registers(OMRON_2SMPB02_REG_STAT, &stat, 1))
        AP_HAL::panic("2SMPB02: unable to read status");

    if (stat & DATA_NOT_READY) {
        _sem->give();
        return;
    }

    _read_raw();
    _calculate();

    // Trigger next conversion
    _dev->write_register(OMRON_2SMPB02_REG_CTRL,
                         T_OVERSAMPLING | P_OVERSAMPLING | 1);

    float temperature = (float) (_temp / 256.0);
    float pressure = (float) (_press / 100.0);

    _copy_to_frontend(_instance, pressure, temperature);
    _sem->give();
}

// Read Raw data values
bool AP_Baro_2SMPB02::_read_raw()
{
    uint8_t buf[OMRON_2SMPB02_RAW_SIZE];

    if (!_dev->read_registers(OMRON_2SMPB02_REG_RAW, buf,
                              OMRON_2SMPB02_RAW_SIZE)) {
        _retry_time = AP_HAL::millis() + 1000;
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);
        return false;
    }

    raw_P = (int32_t)(buf[0] << 16 | buf[1] << 8 | buf[2]);
    raw_T = (int32_t)(buf[3] << 16 | buf[4] << 8 | buf[5]);

    return true;
}

// Calculate Temperature and Pressure in real units.
// See Datasheet page 10 for this formulas calculations.
void AP_Baro_2SMPB02::_calculate()
{
    double dt, tr;

    // Temperature calculations
    dt = raw_T - 0x800000;
    dt = k_BA * k_BA - 4 * k_AA * (k_CA - dt);
    if (dt < 0.0) {
        dt = 0.0;
    }
    if (k_AA == 0.0) {
        tr = 0.0;
    } else {
        tr = (-k_BA - sqrt(dt)) / (2 * k_AA);
    }

    // Pressure calculations
    double dp, pl, po;
    dp = raw_P - 0x800000;
    dp = k_BP * k_BP - 4 * k_AP * (k_CP - dp);
    if (dp < 0.0) {
        pl = 0.0;
    }
    if (k_AP == 0.0) {
        pl = 0.0;
    } else {
        pl = (-k_BP + sqrt(dp)) / (2 * k_AP);
    }

    po = pl / (k_AT * tr * tr + k_BT * tr + k_CT + 1.0);

    _temp = tr;
    _press = po;

    return;
}
