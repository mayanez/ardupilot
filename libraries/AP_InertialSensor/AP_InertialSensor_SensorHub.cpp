#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_SensorHub.h"

#if HAL_SENSORHUB_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;


bool GyroMessageHandler::isValid(GyroMessage::data_t *data)
{
    return !std::isnan(data->gyrox) && !std::isnan(data->gyroy) && !std::isnan(data->gyroz)
        && !std::isinf(data->gyrox) && !std::isinf(data->gyroy) && !std::isinf(data->gyroz);
}

bool AccelMessageHandler::isValid(AccelMessage::data_t *data)
{
    return !std::isnan(data->accelx) && !std::isnan(data->accely) && !std::isnan(data->accelz)
        && !std::isinf(data->accelx) && !std::isinf(data->accely) && !std::isinf(data->accelz);
}

void GyroMessageHandler::handle(GyroMessage::data_t *data)
{
    if (_backend->_sem_gyro->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            uint8_t ins = data->instance;
            auto gyro_info = _backend->_gyro_instance[ins];

            if (gyro_info.registered) {
                if (gyro_info.id != data->id) {
                    // The sensor has changed. Keep same instance & update the id.
                    auto result = _backend->_imu.set_gyro_instance(AP_SensorHub::UPDATE_RATE_HZ, data->id, ins);
                    if (!result) {
                        AP_HAL::panic("ERROR: Could not update gyro instance.\n");
                    }
                }
            } else {
                // A new sensor is detected. Register it.
                ins = _backend->_imu.register_gyro(AP_SensorHub::UPDATE_RATE_HZ, data->id);
                _backend->_gyro_instance[ins].registered = true;
                _backend->_gyro_instance[ins].id = data->id;
                hal.console->printf("GYRO: Register new instance %d, %d\n", ins, data->instance);
            }

            _backend->_gyro[ins].x = data->gyrox;
            _backend->_gyro[ins].y = data->gyroy;
            _backend->_gyro[ins].z = data->gyroz;
            _backend->_last_timestamp[ins] = AP_HAL::micros64();
            _backend->_notify_new_gyro_raw_sample(ins, _backend->_gyro[ins], AP_HAL::micros64());
            _count++;
        } else {
            hal.console->printf("ERROR: GYRO data invalid!\n");
            _error++;
        }

        _backend->_sem_gyro->give();
    }
}

void AccelMessageHandler::handle(AccelMessage::data_t *data)
{
    if (_backend->_sem_accel->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (isValid(data)) {
            uint8_t ins = data->instance;
            auto accel_info = _backend->_accel_instance[ins];

            if (accel_info.registered) {
                if (accel_info.id != data->id) {
                    // The sensor has changed. Keep same instance & update the id.
                    auto result = _backend->_imu.set_accel_instance(AP_SensorHub::UPDATE_RATE_HZ, data->id, ins);
                    if (!result) {
                        AP_HAL::panic("ERROR: Could not update accel instance.\n");
                    }
                }
            } else {
                // A new sensor is detected. Register it.
                ins = _backend->_imu.register_accel(AP_SensorHub::UPDATE_RATE_HZ, data->id);
                _backend->_accel_instance[ins].registered = true;
                _backend->_accel_instance[ins].id = data->id;
                hal.console->printf("ACCEL: Register new instance %d, %d\n", ins, data->instance);
            }

            _backend->_accel[ins].x = data->accelx;
            _backend->_accel[ins].y = data->accely;
            _backend->_accel[ins].z = data->accelz;
            _backend->_last_timestamp[ins] = AP_HAL::micros64();

            _backend->_notify_new_accel_raw_sample(ins, _backend->_accel[ins], AP_HAL::micros64());
            _count++;
        } else {
            hal.console->printf("ERROR: ACCEL data invalid!\n");
            _error++;
        }

        _backend->_sem_accel->give();
    }
}

AP_InertialSensor_SensorHub::AP_InertialSensor_SensorHub(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
    auto gyro_ins = _imu.register_gyro(AP_SensorHub::UPDATE_RATE_HZ, 0);
    auto accel_ins = _imu.register_accel(AP_SensorHub::UPDATE_RATE_HZ, 0);

    _gyro_instance[gyro_ins].registered = true;
    _gyro_instance[gyro_ins].id = 0;

    _accel_instance[accel_ins].registered = true;
    _accel_instance[accel_ins].id = 0;

    _sem_gyro = hal.util->new_semaphore();
    _sem_accel = hal.util->new_semaphore();
    _shub->registerHandler(&_gyro_handler);
    _shub->registerHandler(&_accel_handler);
}

AP_InertialSensor_Backend *AP_InertialSensor_SensorHub::detect(AP_InertialSensor &imu)
{
    AP_InertialSensor_SensorHub *sensor = new AP_InertialSensor_SensorHub(imu);
    if (sensor == nullptr) {
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_SensorHub::update()
{
    if (_sem_gyro->take(HAL_SEMAPHORE_BLOCK_FOREVER) && _sem_accel->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

        for (int i = 0; i < INS_MAX_INSTANCES; i++) {
            if (_gyro_instance[i].registered && _accel_instance[i].registered) {
                update_gyro(i);
                update_accel(i);
            }
        }

        _sem_gyro->give();
        _sem_accel->give();
        return true;
    }

    return false;
}
#endif