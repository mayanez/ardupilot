#include "AP_Compass_SensorHead.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if HAL_SHEAD_ENABLED
void CompassMessageHandler::handle(CompassMessage::data_t *data)
{
    if(_backend->_sem_mag->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _backend->_field.x = data->magx;
        _backend->_field.y = data->magy;
        _backend->_field.z = data->magz;
        _backend->_last_timestamp = AP_HAL::micros64();

            // NOTE: fields already in body frame. no need to rotate.
            _backend->publish_raw_field(
                _backend->_field,
                static_cast<uint32_t>(_backend->_last_timestamp),
                _backend->_instance);

        _backend->correct_field(_backend->_field, _backend->_instance);

        _backend->_sum += _backend->_field;
        _backend->_count++;

        _backend->_sem_mag->give();
    }
}

AP_Compass_SensorHead::AP_Compass_SensorHead(Compass &compass) :
    _count(0),
    AP_Compass_Backend(compass)
{
    _sem_mag = hal.util->new_semaphore();
    _sum.zero();
    _field.zero();
    _instance = register_compass();
    _shead = AP_SensorHead::get_instance();
    _shead->registerHandler(&_handler);
}

void AP_Compass_SensorHead::read()
{
    if (_count == 0) {
        return;
    }

    if (_sem_mag->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _sum /= _count;

        // NOTE: read() is called by frontend. No need to take _sem.
        publish_filtered_field(_sum, _instance);
        _sum.zero();
        _count = 0;

        _sem_mag->give();
    }
}
#endif // HAL_SHEAD_ENABLED