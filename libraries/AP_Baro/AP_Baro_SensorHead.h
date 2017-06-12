#pragma once

#include "AP_Baro_Backend.h"
#include <AP_SensorHead/AP_SensorHead.h>

class AP_Baro_SensorHead;

class BaroMessageHandler : public AP_SensorHead_Handler<BaroMessage> {
public:

    BaroMessageHandler(AP_Baro_SensorHead *backend) :
        _backend(backend)
    {}

    virtual void handle(BaroMessage::data_t *data);
private:
    AP_Baro_SensorHead *_backend;
};

class AP_Baro_SensorHead : public AP_Baro_Backend {
public:
    friend class BaroMessageHandler;

    AP_Baro_SensorHead(AP_Baro &);

    void update() override;


private:
    uint8_t _instance;
    float _pressure;
    float _temperature;
    uint64_t _last_timestamp;

    AP_SensorHead *_shead;
    BaroMessageHandler _handler {this};
    AP_HAL::Semaphore *_sem_baro;
};
