/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#define HACHIDORI_UDP_PORT 5790

#define HACHIDORI_DATA_LEN 32
#define HACHIDORI_MAX_TOS 20

// UDP packet for sensor data and control on hachidori satellite fligt board
struct HACHIDORI_packet {
    uint8_t head;
    uint8_t tos;
    uint8_t data[HACHIDORI_DATA_LEN];
};

// Header byte
#define HACHIDORI_HAEDER_ID	0xb3

// Type of sensor
#define HACHIDORI_TOS_IMU	0
#define HACHIDORI_TOS_MAG	4
#define HACHIDORI_TOS_BARO  8
#define HACHIDORI_TOS_GPS   12
#define HACHIDORI_TOS_BAT   16
#define HACHIDORI_TOS_RANGE 18

// numbers >= 64 are not for sensor but for peripheral or sensor controll
#define HACHIDORI_TOS_PWM   64

#define HACHIDORI_TOS_GPSCMD (64+12)
