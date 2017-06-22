#include <AP_SensorHead/AP_SensorHead.h>
#include <AP_SensorHead/AP_SensorHead_Stream.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
using namespace SensorHead;

/*
 * This example acts as a simple receiver of the SHEAD protocol.
 * Should be used for debugging and timing of the protocol.
 */
class SensorHead_Slave : public AP_HAL::HAL::Callbacks {
public:
    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;
    void receive();

    SensorHead_Slave() :
        shead(AP_SensorHead::init_instance())
    {}

private:
    AP_SensorHead *shead;

    AP_SerialManager serial_manager;

    AP_Baro baro;
    AP_InertialSensor ins;
    Compass compass;
    AP_GPS gps;

    AP_SensorHead_Stream shead_stream;
    AP_HAL::Stream *shead_uart;
};


static SensorHead_Slave slave;

void SensorHead_Slave::setup()
{
    hal.console->printf("SensorHead Slave library example\n");

    AP_BoardConfig{} .init();

    serial_manager.init();
    shead_uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SHEAD, 0);
    if (!shead_uart) {
        AP_HAL::panic("SHEAD UART not found\n");
    }
    shead_stream.registerInputStream(shead_uart);
    shead_stream.registerOutputStream(shead_uart);


    // Init perf counters
    shead->init();

    // Register SensorHead Sensors
    shead->registerSensor(&baro);
    shead->registerSensor(&ins);
    shead->registerSensor(&compass);
    shead->registerSensor(&gps);


    // Initialize Sensors. These will call shead->registerHandler()
    baro.init();
    ins.init(2000);
    gps.init(nullptr, serial_manager);


    if (!compass.init()) {
        AP_HAL::panic("ERROR: COMPASS INIT\n");
    }

    if (!shead_stream.init()) {
        AP_HAL::panic("ERROR: SHEAD STREAM INIT\n");
    }

    hal.scheduler->delay(1000);
    hal.scheduler->register_timer_process(FUNCTOR_BIND(&shead_stream, &AP_SensorHead_Stream::read, void));
}

void SensorHead_Slave::loop()
{
    hal.scheduler->delay_microseconds(100);
}

AP_HAL_MAIN_CALLBACKS(&slave);
