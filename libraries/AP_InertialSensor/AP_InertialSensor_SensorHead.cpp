#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "AP_InertialSensor_SensorHead.h"

extern const AP_HAL::HAL& hal;

#if HAL_SHEAD_ENABLED

bool InertialSensorMessageHandler::isValid(InertialSensorMessage::data_t *data)
{
    return !std::isnan(data->gyrox) && !std::isnan(data->gyroy) && !std::isnan(data->gyroz)
        && !std::isinf(data->gyrox) && !std::isinf(data->gyroy) && !std::isinf(data->gyroz)
        && !std::isnan(data->accelx) && !std::isnan(data->accely) && !std::isnan(data->accelz)
        && !std::isinf(data->accelx) && !std::isinf(data->accely) && !std::isinf(data->accelz);
}

void InertialSensorMessageHandler::handle(InertialSensorMessage::data_t *data)
{
    if (_backend->_sem_ins->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            _backend->_gyro.x = data->gyrox;
            _backend->_gyro.y = data->gyroy;
            _backend->_gyro.z = data->gyroz;
            _backend->_accel.x = data->accelx;
            _backend->_accel.y = data->accely;
            _backend->_accel.z = data->accelz;
            _backend->_last_timestamp = AP_HAL::micros64();
            _backend->_notify_new_accel_raw_sample(_backend->_accel_instance, _backend->_accel, AP_HAL::micros64());
            _backend->_notify_new_gyro_raw_sample(_backend->_gyro_instance, _backend->_gyro, AP_HAL::micros64());
            _count++;
        } else {
            hal.console->printf("ERROR:INS data invalid!\n");
            _error++;
        }

        _backend->_sem_ins->give();
    }
}

AP_InertialSensor_SensorHead::AP_InertialSensor_SensorHead(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
    // TODO: What is the correct id here? Is 0 ok?
    _gyro_instance = _imu.register_gyro(AP_SensorHead::UPDATE_RATE_HZ, 0);
    _accel_instance = _imu.register_accel(AP_SensorHead::UPDATE_RATE_HZ, 0);

    _sem_ins = hal.util->new_semaphore();

    _shead = AP_SensorHead::get_instance();
    _shead->registerHandler(&_handler);
}

AP_InertialSensor_Backend *AP_InertialSensor_SensorHead::detect(AP_InertialSensor &imu)
{
    AP_InertialSensor_SensorHead *sensor = new AP_InertialSensor_SensorHead(imu);
    if (sensor == nullptr) {
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_SensorHead::update()
{
    if (_sem_ins->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        update_gyro(_gyro_instance);
        update_accel(_accel_instance);
        _sem_ins->give();
        return true;
    }

    return false;
}
#endif // HAL_SHEAD_ENABLED