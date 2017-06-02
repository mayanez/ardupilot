#include "AP_SensorHead.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_SensorHead *AP_SensorHead::_instance;

void AP_SensorHead::_decode(std::vector<uint8_t> &buf) {
    SensorHead::msg_t *msgId = reinterpret_cast<SensorHead::msg_t *>(buf.data());
    switch(*msgId){
    case SensorHead::BARO_MSGID:
        SensorHead::baro_msg_t *msg = reinterpret_cast<SensorHead::baro_msg_t *>(buf.data());
        _uart->printf("pressure: %f temp: %f", (double)msg->pressure, (double) msg->temperature);
        //_baro->handle_baro_msg(msg->pressure, msg->temperature);
        break;
    }
}
