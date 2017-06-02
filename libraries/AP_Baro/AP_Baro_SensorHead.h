#pragma once

#include "AP_Baro_Backend.h"
#include <AP_SensorHead/AP_SensorHead.h>

class AP_Baro_SensorHead : public AP_Baro_Backend, public AP_SensorHead_Handler<BaroMessage> {
public:
    AP_Baro_SensorHead(AP_Baro &) {
        _instance = _frontend.register_sensor();
        _sem_baro = hal.util->new_semaphore();
        _shead->registerHandler(this);
    }

    void update() {
        if (_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            _copy_to_frontend(_instance, _pressure, _temperature);

            _frontend.set_external_temperature(_temperature);
            _sem_baro->give();
        }
    }

    // To be called by the AP_SensorHead receive thread.
    virtual void handle(BaroMessage::data_t *data) {
        if (_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            _pressure = data->pressure;
            _temperature = data->temperature - 273.15f;
            _last_timestamp = AP_HAL::micros64();
            _sem_baro->give();
        }
    }

private:
    uint8_t _instance;
    float _pressure;
    float _temperature;
    uint64_t _last_timestamp;

    // TODO: How to set this?
    AP_SensorHead *_shead;
    AP_HAL::Semaphore *_sem_baro;
};
