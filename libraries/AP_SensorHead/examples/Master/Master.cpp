#include <AP_SensorHead/AP_SensorHead.h>
#include <AP_SensorHead/AP_SensorHead_Stream.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
using namespace SensorHead;

// TODO: Should this eventually be a "vehicle"?
class SensorHead_Master : public AP_HAL::HAL::Callbacks {
public:
    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;
    void receive();

    SensorHead_Master() :
        shead(AP_SensorHead::init_instance()),
        param_loader(var_info)
    {}

private:

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    AP_SensorHead *shead;

    AP_SerialManager serial_manager;

    AP_Baro baro;
    AP_InertialSensor ins;
    Compass compass;
    AP_GPS gps;

    AP_SensorHead_Stream shead_stream;

    uint32_t last_baro;
    uint32_t last_compass;
    uint32_t last_gps;
    static const AP_Param::Info var_info[];
    AP_Param param_loader;
    AP_HAL::Stream *shead_uart;
};


static SensorHead_Master master;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
const AP_Param::Info SensorHead_Master::var_info[] = {
    {AP_PARAM_GROUP, "SIM_", 0, &master.sitl, {group_info: SITL::SITL::var_info } },
    {AP_PARAM_GROUP, "GND_", 1, &master.baro, {group_info: AP_Baro::var_info } },
    {AP_PARAM_GROUP, "INS_", 2, &master.ins, {group_info: AP_InertialSensor::var_info } },
    {AP_PARAM_GROUP, "COMPASS_", 3, &master.compass, {group_info: Compass::var_info } },
    AP_VAREND,
};
#endif

void SensorHead_Master::setup()
{
    hal.console->printf("SensorHead Master library example\n");

    AP_BoardConfig{} .init();

    serial_manager.init();
    shead_uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SHEAD, 0);
    if (!shead_uart) {
        AP_HAL::panic("SHEAD UART not found\n");
    }
    shead_stream.registerInputStream(shead_uart);
    shead_stream.registerOutputStream(shead_uart);

    // Register SensorHead Sensors
    shead->registerSensor(&baro);
    shead->registerSensor(&ins);
    shead->registerSensor(&compass);
    shead->registerSensor(&gps);


    // Initialize Sensors.
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
}

void SensorHead_Master::loop()
{
    ins.update(); // NOTE: This implicitly waits for sample. Should be fine
                  // since this sensor is always fastest.
    baro.update();
    compass.read();
    gps.update();

    // NOTE: Because of the wait, when this runs it is guaranteed to have a new
    // sample to write.
    shead_stream.write<InertialSensorMessage>();

    if (last_baro != baro.get_last_update()) {
        shead_stream.write<BaroMessage>();
        last_baro = baro.get_last_update();
    }

    if (last_compass != compass.last_update_ms()) {
        shead_stream.write<CompassMessage>();
        last_compass = compass.last_update_ms();
    }

    if (last_gps != gps.last_message_time_ms()) {
        shead_stream.write<GPSMessage>();
        last_gps = gps.last_message_time_ms();
    }

    hal.scheduler->delay_microseconds(100);
}

AP_HAL_MAIN_CALLBACKS(&master);
