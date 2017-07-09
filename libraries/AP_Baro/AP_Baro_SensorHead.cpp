#include <AP_HAL/AP_HAL.h>

#include "AP_Baro_SensorHead.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#if HAL_SHEAD_ENABLED

bool BaroMessageHandler::isValid(BaroMessage::data_t *data)
{
    return !std::isnan(data->pressure) && !std::isnan(data->temperature);
}

void BaroMessageHandler::handle(BaroMessage::data_t *data)
{
    if (_backend->_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            _backend->_pressure = data->pressure;
            _backend->_temperature = data->temperature;
            _backend->_last_timestamp = AP_HAL::micros64();
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

AP_Baro_SensorHead::AP_Baro_SensorHead(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
    _sem_baro = hal.util->new_semaphore();
    _shead = AP_SensorHead::get_instance();
    _shead->registerHandler(&_handler);
}

void AP_Baro_SensorHead::update()
{
    if (_sem_baro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _copy_to_frontend(_instance, _pressure, _temperature);

        _frontend.set_external_temperature(_temperature);
        _sem_baro->give();
    }
}
#endif // HAL_SHEAD_ENABLED