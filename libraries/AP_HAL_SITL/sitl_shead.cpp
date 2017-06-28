#include <AP_HAL/AP_HAL.h>
#if HAL_SHEAD_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"

#include <AP_Math/AP_Math.h>
#include <SITL/SITL.h>
#include "Scheduler.h"
#include "UARTDriver.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_Baro/AP_Baro_SITL.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Compass/AP_Compass_SITL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_SITL.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

using namespace HALSITL;
extern const AP_HAL::HAL& hal;

/* Simulates a SHEAD Master via UART */

static struct shead_state {
    int master_fd, slave_fd;
    uint32_t last_update;
    uint32_t baro_last_update;
    uint32_t compass_last_update;
    uint32_t ins_last_update;

    uint8_t writeBuffer[SensorHead::Packet::MAX_PACKET_LEN];
} shead_state;

int SITL_State::shead_pipe(void)
{
    int fd[2];
    if (shead_state.slave_fd != 0) {
        return shead_state.slave_fd;
    }

    pipe(fd);
    shead_state.master_fd = fd[1];
    shead_state.slave_fd = fd[0];
    HALSITL::UARTDriver::_set_nonblocking(shead_state.master_fd);
    HALSITL::UARTDriver::_set_nonblocking(shead_state.slave_fd);
    return shead_state.slave_fd;
}

void SITL_State::_shead_init()
{
    _shead = new AP_SensorHead();
    _shead->registerSensor(_barometer);
    _shead->registerSensor(_compass);
    _shead->registerSensor(_ins);

    _barometer->sitl_init();
    _barometer->_add_backend(new AP_Baro_SITL(*_barometer));
    _barometer->calibrate();

    fprintf(stdout, "SHEAD SITL INIT\n");
}

void SITL_State::_shead_write(const uint8_t *p, uint16_t size, uint8_t instance)
{
    while (size--) {
        if (instance == 0 && shead_state.master_fd != 0) {
            write(shead_state.master_fd, p, 1);
        }
        p++;
    }
}

void SITL_State::_shead_update()
{
    // 1kHz
    if (AP_HAL::micros() - shead_state.last_update < 1000) {
        return;
    }
    shead_state.last_update = AP_HAL::micros();

    _barometer->update();

    if (shead_state.baro_last_update != _barometer->get_last_update()) {
        _shead->write<BaroMessage>(&shead_state.writeBuffer[0], sizeof(shead_state.writeBuffer));
        _shead_write(&shead_state.writeBuffer[0], BaroMessage::PACKET_LENGTH, 0);
        shead_state.baro_last_update = _barometer->get_last_update();
    }

}
#endif