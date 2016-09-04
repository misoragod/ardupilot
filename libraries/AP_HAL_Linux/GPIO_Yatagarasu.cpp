#include <AP_Common/AP_Common.h>

#include "GPIO_Yatagarasu.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_YATAGARASU

#define BOARD96_HIKEY

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
#if defined(BOARD96_HIKEY)
    [YATAGARASU_GPIO_A] =     488,
    [YATAGARASU_GPIO_B] =     489,
    [YATAGARASU_GPIO_C] =     490,
    [YATAGARASU_GPIO_D] =     491,
    [YATAGARASU_GPIO_E] =     492,
    [YATAGARASU_GPIO_F] =     415,
    [YATAGARASU_GPIO_G] =     463,
    [YATAGARASU_GPIO_H] =     495,
    [YATAGARASU_GPIO_I] =     426,
    [YATAGARASU_GPIO_J] =     433,
    [YATAGARASU_GPIO_K] =     427,
    [YATAGARASU_GPIO_L] =     434,
    [YATAGARASU_EXT_GPIO_0] = 336,
    [YATAGARASU_EXT_GPIO_1] = 337,
    [YATAGARASU_EXT_GPIO_2] = 338,
    [YATAGARASU_EXT_GPIO_3] = 339,
    [YATAGARASU_EXT_GPIO_4] = 340,
    [YATAGARASU_EXT_GPIO_5] = 341,
    [YATAGARASU_EXT_GPIO_6] = 342,
    [YATAGARASU_EXT_GPIO_7] = 343,
    [YATAGARASU_EXT_GPIO_8] = 344,
    [YATAGARASU_EXT_GPIO_9] = 345,
    [YATAGARASU_EXT_GPIO_A] = 346,
    [YATAGARASU_EXT_GPIO_B] = 347,
    [YATAGARASU_EXT_GPIO_C] = 348,
    [YATAGARASU_EXT_GPIO_D] = 349,
    [YATAGARASU_EXT_GPIO_E] = 350,
    [YATAGARASU_EXT_GPIO_F] = 351,
#else
#error "unknown 96board: add GPIO pin numbers for your 96board"
#endif
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _YATAGARASU_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _YATAGARASU_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_hikey");

#endif
