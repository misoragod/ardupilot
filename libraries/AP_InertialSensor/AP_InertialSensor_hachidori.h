/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Linux/Semaphores.h>
#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

#include <AP_HAL/utility/RingBuffer.h>

#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>

class AP_InertialSensor_HACHIDORI : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_HACHIDORI(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // detect the sensor port
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool _init();

    /* Read a single sample */
    void _timer_update();

    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    // The default rotation for the IMU, its value depends on how the IMU is
    // placed by default on the system
    enum Rotation _default_rotation;
};

class AP_Compass_HACHIDORI;
class AP_Baro_HACHIDORI;
class AP_RangeFinder_HACHIDORI;
class AnalogIn_HACHIDORI;

class HACHIDORI_PORT
{
    friend AP_InertialSensor_HACHIDORI;
    friend AP_Compass_HACHIDORI;
    friend AP_Baro_HACHIDORI;
    friend AP_RangeFinder_HACHIDORI;
    friend AnalogIn_HACHIDORI;

public:
    static void open_port();
    static int get_port();
    static bool check_port(struct HACHIDORI_packet&, bool);
    static bool send_port(struct HACHIDORI_packet&);
    static int read_gps(uint8_t *buf, size_t n);

private:
    // datagram socket
    static int fd;
    // client address
    static in_addr_t in_addr;
    static bool in_addr_set;
    // packet buffer
    static struct HACHIDORI_packet buf[HACHIDORI_MAX_TOS];
    // sensor packet status
    static bool updated[HACHIDORI_MAX_TOS];
    static bool taken;
    // ring buffer for uart (GPS)
    static ByteBuffer *gpsbuf;
    // lock for send_port
    static Linux::Semaphore send_semaphore;
};

#endif // CONFIG_HAL_BOARD_SUBTYPE
