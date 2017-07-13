#include <AP_HAL/AP_HAL.h>
#include "AP_Baro_SensorHub.h"

#if HAL_SENSORHUB_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

bool BaroMessageHandler::isValid(BaroMessage::data_t *data)
{
    return !std::isnan(data->pressure) && !std::isnan(data->temperature);
}

void BaroMessageHandler::handle(BaroMessage::data_t *data)
{
    if (_backend->_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            auto ins = data->instance;
            auto instance_registered = _backend->_instance[ins];

            if (!instance_registered) {
                ins = _backend->_frontend.register_sensor();
                _backend->_instance[ins] = true;
            }

            _backend->_pressure[ins] = data->pressure;
            _backend->_temperature[ins] = data->temperature;
            _backend->_last_timestamp[ins] = AP_HAL::micros64();
            _backend->publish_raw(ins, _backend->_pressure[ins], _backend->_temperature[ins]);
            _count++;
        } else {
            // A packet is successfully decoded, however its data is not valid.
            // This most likely due to CRC collision.
            // TODO: Appropriately log this error.
            hal.console->printf("ERROR:Baro data invalid!\n");
            _error++;
        }
        _backend->_sem_baro->give();
    }
}

AP_Baro_SensorHub::AP_Baro_SensorHub(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _sem_baro = hal.util->new_semaphore();

    auto ins = _frontend.register_sensor();
    _instance[ins] = true;

    _shub->registerHandler(&_handler);
}

void AP_Baro_SensorHub::update()
{
    if (_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

        for (int i = 0; i < BARO_MAX_INSTANCES; i++) {
            if (_instance[i]) {
                _copy_to_frontend(i, _pressure[i], _temperature[i]);
            }
        }

        _sem_baro->give();
    }
}
#endif