#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#include <AP_SensorHead/AP_SensorHead.h>

class AP_Compass_SensorHead;

class CompassMessageHandler : public AP_SensorHead_Handler<CompassMessage> {
public:

    CompassMessageHandler(AP_Compass_SensorHead *backend) :
        _backend(backend)
    {}

    virtual void handle(CompassMessage::data_t *data);

private:
    AP_Compass_SensorHead *_backend;
};

class AP_Compass_SensorHead : public AP_Compass_Backend {
public:
    friend class CompassMessageHandler;

    void read(void) override;

    AP_Compass_SensorHead(Compass &compass);

    void handle_mag_msg(Vector3f &msg) {}

private:
    uint8_t _instance;
    Vector3f _field;
    Vector3f _sum;
    uint32_t _count;
    uint64_t _last_timestamp;

    AP_SensorHead *_shead;
    CompassMessageHandler _handler {this};
    AP_HAL::Semaphore *_sem_mag;
};
