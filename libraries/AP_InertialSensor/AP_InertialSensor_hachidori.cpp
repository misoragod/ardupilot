/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_HACHIDORI

#include "AP_InertialSensor_hachidori.h"

#include <errno.h>

const extern AP_HAL::HAL& hal;

// constructor
AP_InertialSensor_HACHIDORI::AP_InertialSensor_HACHIDORI(AP_InertialSensor &imu)
    : AP_InertialSensor_Backend(imu)
    , _default_rotation(ROTATION_NONE)
{
}

// detect sensor port
AP_InertialSensor_Backend *AP_InertialSensor_HACHIDORI::detect(AP_InertialSensor &_imu)
{
    HACHIDORI_PORT::open_port();
    AP_InertialSensor_HACHIDORI *sensor = new AP_InertialSensor_HACHIDORI(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_InertialSensor_HACHIDORI::_init(void)
{

    _gyro_instance = _imu.register_gyro(1000, 1);
    _accel_instance = _imu.register_accel(1000, 1);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_HACHIDORI::_timer_update, void));
    return true;
}

void AP_InertialSensor_HACHIDORI::_timer_update(void)
{
    while (true) {
        struct HACHIDORI_packet pkt;
        if (!HACHIDORI_PORT::check_port(pkt, true)) {
            break;
        }

        if (pkt.tos == HACHIDORI_TOS_IMU) {
            uint8_t *rx = pkt.data;
            Vector3f accel, gyro;

            // hachidori sends data with little endian format
            le32_t *fp = reinterpret_cast<le32_t *>(rx);
            union { float f; uint32_t u32; } x, y, z;
            // accel
            x.u32 = le32toh(fp[0]);
            y.u32 = le32toh(fp[1]);
            z.u32 = le32toh(fp[2]);

            accel = Vector3f(x.f, y.f, z.f);
            accel.rotate(_default_rotation);
            _rotate_and_correct_accel(_accel_instance, accel);
            _notify_new_accel_raw_sample(_accel_instance, accel);

            // gyro
            x.u32 = le32toh(fp[3]);
            y.u32 = le32toh(fp[4]);
            z.u32 = le32toh(fp[5]);
            gyro = Vector3f(x.f, y.f, z.f);
            gyro.rotate(_default_rotation);
            _rotate_and_correct_gyro(_gyro_instance, gyro);
            _notify_new_gyro_raw_sample(_gyro_instance, gyro);
        }
    }
}

bool AP_InertialSensor_HACHIDORI::update(void)
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}

// HACHIDORI_PORT
// static members
int HACHIDORI_PORT::fd = -1;
in_addr_t HACHIDORI_PORT::in_addr = INADDR_ANY;
bool HACHIDORI_PORT::in_addr_set = false;
struct HACHIDORI_packet HACHIDORI_PORT::buf[HACHIDORI_MAX_TOS];
bool HACHIDORI_PORT::updated[HACHIDORI_MAX_TOS];
bool HACHIDORI_PORT::taken = false;
ByteBuffer *HACHIDORI_PORT::gpsbuf;
Linux::Semaphore HACHIDORI_PORT::send_semaphore;

#define GPSBUF_SIZE 2048

void HACHIDORI_PORT::open_port(void)
{
    if (fd < 0) {
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) {
            AP_HAL::panic("unable to get socket for sensor port");
        }

        int option = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

        struct sockaddr_in saddr;
        saddr.sin_family = AF_INET;
        saddr.sin_addr.s_addr = htonl(INADDR_ANY);
        saddr.sin_port = htons(HACHIDORI_UDP_PORT);
        if (bind(fd, (struct sockaddr *) &saddr, sizeof(saddr)) < 0) {
            AP_HAL::panic("Unable to bind socket to local sensor port");
        }

        option = 1;
        if (ioctl(fd, FIONBIO, &option) < 0) {
            AP_HAL::panic("Unable to make socket non-blocking");
        }
        for (int i = 0; i < HACHIDORI_MAX_TOS; i++) {
            updated[i] = false;
        }

        gpsbuf = new ByteBuffer(GPSBUF_SIZE);
    }
}

int HACHIDORI_PORT::get_port(void)
{
    return fd;
}

// Assume that check_port is called by IMU process after taken and by
// timer processes before taken.  No lock required ATM.
bool HACHIDORI_PORT::check_port(struct HACHIDORI_packet &pkt, bool take)
{
    if (taken && !take) {
        return false;
    } else if (take) {
        taken = true;
    }

    static int wdcount = 0;
    struct sockaddr_in caddr;
    socklen_t clilen = sizeof(caddr);
    memset(&caddr, 0, clilen);
    int n = recvfrom(fd, &pkt, sizeof(pkt), 0, (struct sockaddr *) &caddr,
                     &clilen);
    if (n < 0 || (size_t)n < sizeof(pkt)) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Maybe lost connection
            if (++wdcount > 2000) {
                AP_HAL::panic("can't get any activities from frame");
            }
            goto fail;
        }
        // TODO: report or diagnose i/o error
        goto fail;
    }
    wdcount = 0;

    // TODO: Check client address here
    if (!in_addr_set) {
        in_addr = caddr.sin_addr.s_addr;
        if (in_addr == INADDR_ANY) {
            AP_HAL::panic("can't get client ip");
        }
        in_addr_set = true;
    }

    if (pkt.head != HACHIDORI_HAEDER_ID) {
        goto fail;
    }
    if (pkt.tos == HACHIDORI_TOS_IMU) {
        ; // IMU packets are processed directly
    } else if (pkt.tos == HACHIDORI_TOS_GPS) {
        uint32_t len = pkt.data[0];
        if (len) {
            if (gpsbuf->write(&pkt.data[1], len) != len) {
                hal.console->printf("Hachidori: error writing gps buf\n");
            }
        }
    } else if (pkt.tos < HACHIDORI_MAX_TOS) {
        memcpy(&buf[pkt.tos], &pkt, sizeof(pkt));
        updated[pkt.tos] = true;
    }

    return true;

 fail:
    return false;
}

bool HACHIDORI_PORT::send_port(struct HACHIDORI_packet& pkt)
{
    if (!in_addr_set) {
        return false;
    }

    if (!send_semaphore.take(0)) {
        return false;
    }

    struct sockaddr_in caddr;
    socklen_t clilen = sizeof(caddr);
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr = in_addr;
    caddr.sin_port = htons(HACHIDORI_UDP_PORT);

    int n = sendto(fd, &pkt, sizeof(pkt), 0, (struct sockaddr *) &caddr,
                   clilen);
    if (n < 0) {
        AP_HAL::panic("sendto err %d", errno);
    }
    if (n != sizeof(pkt)) {
        AP_HAL::panic("sendto only send %d bytes", n);
    }

    send_semaphore.give();

    return true;
}

int HACHIDORI_PORT::read_gps(uint8_t *buf, size_t n)
{
    if (!gpsbuf) {
        return 0;
    }

    return gpsbuf->read(buf, n);
}

#endif
