#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_SENSORHEAD
#include "AP_Baro_SensorHead.h"

#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

AP_Baro_SensorHead::AP_Baro_SensorHead(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
    _sensorhead = AP_SensorHead::get_instance();
    _sem_baro = hal.util->new_semaphore();
}

#if CONFIG_SENSORHEAD == SENSORHEAD_MASTER
void AP_Baro_SensorHead::update(void)
{
    if(_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        SensorHead::baro_msg_t baro_msg;
        baro_msg.pressure = _frontend.get_pressure();
        baro_msg.temperature = _frontend.get_temperature();
        _sensorhead->send<SensorHead::baro_msg_t>(&baro_msg);
        _sem->give();
    }
}

void AP_Baro_SensorHead::handle_baro_msg(float pressure, float temperature) {}
#elif CONFIG_SENSORHEAD == SENSORHEAD_SLAVE
// These will be similar to AP_Baro_UAVCAN

void AP_Baro_SensorHead::update(void) {
}

void AP_Baro_SensorHead::handle_baro_msg(float pressure, float temperature) {

}
#endif

#endif
