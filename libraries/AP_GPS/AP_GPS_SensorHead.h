#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SensorHead/AP_SensorHead.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if HAL_SHEAD_ENABLED
class AP_GPS_SensorHead;

class GPSMessageHandler : public AP_SensorHead_Handler<GPSMessage> {
public:

    GPSMessageHandler(AP_GPS_SensorHead *backend) :
        _backend(backend)
    {}

    virtual void handle(GPSMessage::data_t *data);
private:
    AP_GPS_SensorHead *_backend;
};


class AP_GPS_SensorHead : public AP_GPS_Backend {
public:
    friend class GPSMessageHandler;

    AP_GPS_SensorHead(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read() override;

    void handle_gnss_msg(const AP_GPS::GPS_State &msg) {};

    const char *name() const override { return "SHEAD"; }

private:
    bool _new_data;

    AP_GPS::GPS_State _interm_state;

    AP_SensorHead *_shead;
    GPSMessageHandler _handler {this};
    AP_HAL::Semaphore *_sem_gnss;
};
#endif // HAL_SHEAD_ENABLED