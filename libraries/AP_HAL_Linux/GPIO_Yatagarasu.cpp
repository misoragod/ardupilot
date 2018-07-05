#include <AP_Common/AP_Common.h>

#include "GPIO_Yatagarasu.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_YATAGARASU

#define BOARD96_HIKEY

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
#if defined(BOARD96_HIKEY)
    [YATAGARASU_GPIO_A] =     36,
    [YATAGARASU_GPIO_B] =     12,
    [YATAGARASU_GPIO_C] =     13,
    [YATAGARASU_GPIO_D] =     69,
    [YATAGARASU_GPIO_E] =     115,
    [YATAGARASU_GPIO_F] =     507,
    [YATAGARASU_GPIO_G] =     24,
    [YATAGARASU_GPIO_H] =     25,
    [YATAGARASU_GPIO_I] =     35,
    [YATAGARASU_GPIO_J] =     34,
    [YATAGARASU_GPIO_K] =     28,
    [YATAGARASU_GPIO_L] =     38,
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
