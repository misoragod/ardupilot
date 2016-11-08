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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCInput_XBUS.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
// glibc tcgetattr/tcsetattr can't handle 250k baud speed.  We use termios2
// stuff to set that speed.  termios.h can't be included here to avoid
// redefinition errors.
#include <asm/termbits.h>
extern "C" {
    int tcgetattr(int fd, struct termios *termios_p);
    int tcsetattr(int fd, int optional_actions,
                  const struct termios *termios_p);
    int tcflush(int fd, int optional_actions);
}

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace Linux;

// xbus frame input state
enum { WAIT_START, READ_HEADER, READ_CHANNEL };

RCInput_XBUS::RCInput_XBUS(const char *path)
{
    _fd = open(path, O_RDONLY|O_NOCTTY|O_NONBLOCK|O_NDELAY);
    if (_fd < 0) {
        AP_HAL::panic("RCInput_XBUS: Error opening '%s': %s",
		      path, strerror(errno));
    }
}

RCInput_XBUS::~RCInput_XBUS()
{
    close(_fd);
}

void RCInput_XBUS::init()
{
    struct termios options;

    tcgetattr(_fd, &options);

    // 8-bit, no parity, 1-stop bit
    options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    options.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);
    options.c_oflag &= ~OPOST;

    if (tcsetattr(_fd, TCSANOW, &options) != 0) {
        AP_HAL::panic("RCInput_XBUS: error configuring device: %s",
		      strerror(errno));
    }

    tcflush(_fd, TCIOFLUSH);

    struct termios2 options2;
    ioctl(_fd, TCGETS2, &options2);
    options2.c_cflag &= ~CBAUD;
    options2.c_cflag |= 0010000; // BOTHER
    options2.c_ispeed = 250000;
    options2.c_ospeed = 250000;
    if (ioctl(_fd, TCSETS2, &options2) != 0) {
        AP_HAL::panic("RCInput_XBUS: can't set 250k buad rate: %s",
		      strerror(errno));
    }

    _state = WAIT_START;
    _offset = 0;
}

static uint8_t crc8_array[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/*
    XBUS packet is a byte sequence which looks like:
    command(0xa4), length(2+4*(1-50)), key(0), type(0),
    ch-id(1-50), ch-func(0), ch-data-high, ch-data-low,
    repeated ch-* stuff,
    crc8 (x^8+x^5+x^4+1) from key to just before crc8

    Some XBUS recievers send the packet which has odd key, type
    and ch-func.  If you want to check these bytes, define macros
    XBUS_CHECK_KEY
    XBUS_CHECK_TYPE
    XBUS_CHECK_CHANNEL_FANCTION
    so as to ignore that packet.
*/

void RCInput_XBUS::_timer_tick()
{
    uint8_t c;
    ssize_t n;

    while (1) {
        if ((n = ::read(_fd, &c, 1)) <= 0)
            return;

        switch (_state) {
        case WAIT_START:
            if (c == 0xa4) {
                ++_offset;
                _state = READ_HEADER;
                for (int i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++) {
                    _chdata[i] = 0x7fff;
                }
                _crc = 0;
                _crc = crc8_array[(_crc ^ c) & 0xff];
            }
            break;
	    case READ_HEADER:
            if (_offset == 1) {
                // Get length
                _length = c;
                ++_offset;
                _crc = crc8_array[(_crc ^ c) & 0xff];
            } else if (_offset == 2) {
#if defined(XBUS_CHECK_KEY)
                // Check if key == 0
                if (c != 0) {
                    _state = WAIT_START;
                    _offset = 0;
                    continue;
                }
#endif
                --_length;
                ++_offset;
                _crc = crc8_array[(_crc ^ c) & 0xff];
            } else if (_offset == 3) {
#if defined(XBUS_CHECK_TYPE)
                // Check if type == 0
                if (c != 0) {
                    _state = WAIT_START;
                    _offset = 0;
                    continue;
                }
#endif
                --_length;
                ++_offset;
                _crc = crc8_array[(_crc ^ c) & 0xff];
               _state = READ_CHANNEL;
            }
            break;
        case READ_CHANNEL:
            if (_length == 0) {
                _state = WAIT_START;
                _offset = 0;
                // Check crc
                if (_crc == c) {
                    uint16_t values[LINUX_RC_INPUT_NUM_CHANNELS];
                    for (int i = 0; i < LINUX_RC_INPUT_NUM_CHANNELS; i++) {
                        // 0 -> 800micro, 0xffff -> 2200micro
                        values[i] = 800 + ((2200 - 800)*_chdata[i])/0xffff;
                    }
                    _update_periods(values, LINUX_RC_INPUT_NUM_CHANNELS);
                }
                continue;
            }
            switch (_offset % 4)
                {
                case 0:
                    _chid = c;
                    break;
                case 1:
#if defined(XBUS_CHECK_CHANNEL_FANCTION)
                    // Check if channel fuction field is 0
                    if (c != 0) {
                        _state = WAIT_START;
                        _offset = 0;
                        continue;
                    }
#endif
                    break;
                case 2:
                    _chhigh = c;
                    break;
                case 3:
                    if (_chid > 0 && _chid <= LINUX_RC_INPUT_NUM_CHANNELS) {
                        _chdata[_chid-1] = (_chhigh << 8) + c;
                    }
                    break;
                default:
                    break;
                }
            --_length;
            ++_offset;
            _crc = crc8_array[(_crc ^ c) & 0xff];
            break;
        default:
            break;
        }
    }
}
#endif
