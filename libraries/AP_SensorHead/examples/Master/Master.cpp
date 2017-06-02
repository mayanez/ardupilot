#include <AP_SensorHead/AP_SensorHead.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
using namespace SensorHead;

// TODO: Should this eventually be a "vehicle"?
class SensorHead_Master : public AP_HAL::HAL::Callbacks {
public:
    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    SITL::SITL sitl;

    AP_Scheduler scheduler;
    AP_SensorHead sensorhead;
    AP_Baro baro;
    AP_InertialSensor ins;

    static const AP_Scheduler::Task scheduler_tasks[];

    void load_parameters(void);
    void ins_update(void);
    void baro_update(void);
    void baro_accumulate(void);
};

static SensorHead_Master master;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(SensorHead_Master, &master, func, _interval_ticks, _max_time_micros)

const AP_Scheduler::Task SensorHead_Master::scheduler_tasks[] = {
    SCHED_TASK(ins_update,             50,   1000),
    SCHED_TASK(baro_accumulate,        50,   1000),
    SCHED_TASK(baro_update,            10,   1000),
};

void SensorHead_Master::load_parameters(void) {
    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad var table\n");
        AP_HAL::panic("bad var table");
    }

    AP_Param::load_all(false);
}

void SensorHead_Master::setup()
{

    hal.uartD->begin(57600);
    hal.console->printf("SensorHead Master library test\n");

    AP_BoardConfig{}.init();
    load_parameters();

    shead.baro(&baro).ins(&ins).stream(hal.uartD);

    ins.init(scheduler.get_loop_rate_hz());

    hal.scheduler->delay(1000);

    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

void SensorHead_Master::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all tasks that fit in allotted time
    scheduler.run(1000);
}

void SensorHead_Master::ins_update() {
    ins.update();
    shead.ins_update();
}
void SensorHead_Master::baro_update() {
    baro.update();
    shead.baro_update();
}

void SensorHead_Master::baro_accumulate() {
    baro.accumulate();
}

AP_HAL_MAIN_CALLBACKS(&master);
