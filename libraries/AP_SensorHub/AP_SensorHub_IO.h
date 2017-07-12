#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_SENSORHUB_ENABLED

#include "Protocol.h"

using namespace SensorHub;

class AP_SensorHub;

/* Base class for IO Channel */
class AP_SensorHub_IO {
public:
    virtual void read() = 0;
    virtual void write(Packet::packet_t *packet, size_t len) = 0;

    void setSensorHub(AP_SensorHub *shub) {
        _shub = shub;
    }
protected:
    AP_SensorHub *_shub;
};
#endif
