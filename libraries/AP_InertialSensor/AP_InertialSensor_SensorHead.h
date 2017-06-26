#pragma once

#include <AP_SensorHead/AP_SensorHead.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#if HAL_SHEAD_ENABLED
class AP_InertialSensor_SensorHead;

class InertialSensorMessageHandler : public AP_SensorHead_Handler<InertialSensorMessage> {
public:

    InertialSensorMessageHandler(AP_InertialSensor_SensorHead *backend) :
        _backend(backend)
    {}

    virtual void handle(InertialSensorMessage::data_t *data);
private:
    AP_InertialSensor_SensorHead *_backend;
};


class AP_InertialSensor_SensorHead : public AP_InertialSensor_Backend
{
public:
    friend class InertialSensorMessageHandler;

    AP_InertialSensor_SensorHead(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    Vector3f _gyro;
    Vector3f _accel;
    uint64_t _last_timestamp;

    AP_SensorHead *_shead;
    InertialSensorMessageHandler _handler {this};
    AP_HAL::Semaphore *_sem_ins;
};
#endif // HAL_SHEAD_ENABLED