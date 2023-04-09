

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>


#include <drivers/differential_pressure/asp5033driver/ASP5033Driver.hpp>//mine
#include <uORB/topics/differential_pressure.h>//mine



using namespace time_literals;

static constexpr uint32_t SCHEDULE_INTERVAL{100_ms};	/**< The schedule interval in usec (10 Hz) */



class ASPDeni : public ModuleBase<ASPDeni>, public px4::ScheduledWorkItem, public ASP5033Driver
{
public:

	ASPDeni();

	~ASPDeni() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:

	void Run() override;

	uORB::Publication<differential_pressure_s> _differential_pressure_pub {ORB_ID(differential_pressure)};//mine
	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};//mine
	differential_pressure_s _pressure; //mine


	hrt_abstime _time_last_airspeed_update[0] {};
	hrt_abstime _time_now_usec{0};


	perf_counter_t _perf_elapsed{};


	void 		poll_topics(); /**< poll all topics required beside airspeed (e.g. current temperature) */
	void 		select_airspeed_and_publish(); /**< select airspeed sensor (or groundspeed-windspeed) */

};

ASPDeni::ASPDeni():
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{

	_perf_elapsed = perf_alloc(PC_ELAPSED, MODULE_NAME": elapsed");

	_differential_pressure_pub.advertise(); //mine

}

ASPDeni::~ASPDeni()
{
	ScheduleClear();

	perf_free(_perf_elapsed);
}

int
ASPDeni::task_spawn(int argc, char *argv[])
{
	ASPDeni *dev = new ASPDeni();

	// check if the trampoline is called for the first time
	if (!dev) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(dev);

	dev->ScheduleOnInterval(SCHEDULE_INTERVAL, 10000);
	_task_id = task_id_is_work_queue;
	return PX4_OK;

}


void
ASPDeni::Run()
{
	//PX4_INFO("ASPDENI");
	_time_now_usec = hrt_absolute_time(); // hrt time of the current cycle
	if (_time_now_usec < 2_s) {
		return;
	}

	perf_begin(_perf_elapsed);

	poll_topics();
	select_airspeed_and_publish();

	perf_end(_perf_elapsed);

	if (should_exit()) {
		exit_and_cleanup();
	}
}


void ASPDeni::poll_topics()
{

	_differential_pressure_sub.update(&_pressure); //mine



}



void ASPDeni::select_airspeed_and_publish()
{
	differential_pressure_s _diff_pressure; //mine
	_diff_pressure.timestamp_sample = 0;
	_diff_pressure.device_id= 200;
	_diff_pressure.differential_pressure_pa = differential_pressure_d(); //float
	_diff_pressure.temperature = temperature_d();   //float
	_diff_pressure.error_count = 0;

	_differential_pressure_pub.publish(_diff_pressure);


}

int ASPDeni::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = ASPDeni::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int ASPDeni::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR( ASP5033 sensor test)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("aspdeni", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int aspdeni_main(int argc, char *argv[])
{
	PX4_INFO("ASP5033 started.");
	return ASPDeni::main(argc, argv);
}
