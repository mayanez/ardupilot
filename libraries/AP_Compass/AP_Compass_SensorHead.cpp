#include "AP_Compass_SensorHead.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if HAL_SHEAD_ENABLED
bool CompassMessageHandler::isValid(CompassMessage::data_t *data)
{
    return !std::isnan(data->magx) && !std::isnan(data->magy) && !std::isnan(data->magz)
        && !std::isinf(data->magx) && !std::isinf(data->magy) && !std::isinf(data->magz)
        ;//&& !(fabsf(data->magx) < FLT_EPSILON) && !(fabsf(data->magy) < FLT_EPSILON) && !(fabsf(data->magz) < FLT_EPSILON);
}

void CompassMessageHandler::handle(CompassMessage::data_t *data)
{
    if(_backend->_sem_mag->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            _backend->_field.x = data->magx;
            _backend->_field.y = data->magy;
            _backend->_field.z = data->magz;
            _backend->_last_timestamp = AP_HAL::micros64();
            _count++;
        } else {
            // A packet is successfully decoded, however its data is not valid.
            // This most likely due to CRC collision.
            // TODO: Appropriately log this error.
            hal.console->printf("ERROR:Compass data invalid!\n");
            _error++;
        }
        _backend->_sem_mag->give();
    }
}

AP_Compass_SensorHead::AP_Compass_SensorHead(Compass &compass) :
    AP_Compass_Backend(compass)
{
    _sem_mag = hal.util->new_semaphore();
    _field.zero();
    _instance = register_compass();
    _shead = AP_SensorHead::get_instance();
    _shead->registerHandler(&_handler);
}

void AP_Compass_SensorHead::read()
{
    if (_sem_mag->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        publish_filtered_field(_field, _instance);
        _sem_mag->give();
    }
}
#endif // HAL_SHEAD_ENABLED