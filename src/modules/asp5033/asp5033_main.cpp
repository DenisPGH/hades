// DENIS
#include <poll.h>
//#include "ASP5033.hpp"
#include <uORB/topics/differential_pressure.h>

#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <drivers/differential_pressure/asp5033driver/ASP5033Driver.hpp>

using namespace time_literals;

static constexpr uint32_t SCHEDULE_INTERVAL{100_ms};	/**< The schedule interval in usec (10 Hz) */



class Asp5033 : public ModuleBase<Asp5033>, public ASP5033Driver
	 //public px4::ScheduledWorkItem
{
public:

	Asp5033();

	//~Asp5033() override;


	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static Asp5033 *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void run() override; //override


	uORB::Publication<differential_pressure_s> _differential_pressure_pub {ORB_ID(differential_pressure)};

	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};

	float temp=30.33;
	differential_pressure_s _diff_pressure {};
};



Asp5033::Asp5033()
 	//ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_differential_pressure_pub.advertise();
}

// Asp5033::~Asp5033()
// {
// 	//ScheduleClear();
// }



int
Asp5033::task_spawn(int argc, char *argv[])
{
	// Entry point function pointer
	// px4_main_t entry_point=(px4_main_t)&run_trampoline;
	// int _task_id = px4_task_spawn_cmd("asp5033", SCHED_DEFAULT,SCHED_PRIORITY_DEFAULT,
	//  1500,entry_point,(char *const *)argv);
	// // //_task_id = task_id_is_work_queue;
	// return _task_id;

	Asp5033 *dev = new Asp5033();

	// check if the trampoline is called for the first time
	if (!dev) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(dev);

	//dev->ScheduleOnInterval(SCHEDULE_INTERVAL, 10000);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

int
Asp5033::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = Asp5033::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int
Asp5033::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(### Description ASP5033 test module )DESCR_STR");

	PRINT_MODULE_USAGE_NAME("differential_pressure", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}







void
Asp5033::run()
{
	PX4_INFO("Running");

	//_differential_pressure_sub.update(&_diff_pressure);
	//_diff_pressure.timestamp = _time_now_usec_r;
	_diff_pressure.timestamp_sample = 0;
	_diff_pressure.device_id= 200;
	_diff_pressure.differential_pressure_pa = differential_pressure_d(); //float
	_diff_pressure.temperature = temp;   //float
	_diff_pressure.error_count = 0;

	_differential_pressure_pub.publish(_diff_pressure);

	if (should_exit()) {
		exit_and_cleanup();
	}
	return;

}








extern "C" __EXPORT int asp5033_main(int argc, char *argv[])
{	PX4_INFO("asp5033=========================");
	return Asp5033::main(argc,argv);

}
