#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_SENSORHEAD
#include "AP_Baro_Backend.h"
#include <AP_SensorHead/AP_SensorHead.h>

class AP_Baro_SensorHead : public AP_Baro_Backend {
public:
    AP_Baro_SensorHead(AP_Baro &);

    virtual void update(void) override;
    virtual void handle_baro_msg(float pressure, float temperature) override;

private:
    uint8_t _instance;
    float _pressure;
    float _temperature;
    uint64_t _last_timestamp;

    AP_HAL::Semaphore *_sem_baro;
    AP_SensorHead *_sensorhead;
};
#endif
