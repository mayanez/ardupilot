#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

namespace SensorHead {
    enum msg_t {
        BARO_MSGID = 1,
    };

    struct PACKED {
        float pressure;
        float temperature;
    } typedef baro_msg_t;
}
