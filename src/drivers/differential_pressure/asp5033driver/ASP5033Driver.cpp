

#include "ASP5033Driver.hpp"

ASP5033Driver::ASP5033Driver(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr) //param

{
	_differential_pressure_pub.advertise(); //param
	// initialise parameters
	update_params();  //param
}

ASP5033Driver::~ASP5033Driver()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);
}

int ASP5033Driver::probe()
{
	//PX4_INFO("probe func");
	//mavlink_log_info(&_mavlink_log_pub,"probe func");
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, nullptr, 0);
	//int ret = transfer(&REG_CMD_ASP5033, 1, nullptr, 0);

	return ret;
}

int ASP5033Driver::init()
{
	int ret = I2C::init();
	//PX4_INFO("init func");
	//mavlink_log_info(&_mavlink_log_pub,"init func");

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

int ASP5033Driver::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd = CMD_MEASURE_ASP5033;
	//uint8_t cmd=REG_CMD_ASP5033;
	//PX4_INFO("measure func");
	//mavlink_log_info(&_mavlink_log_pub,"meas func");
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int ASP5033Driver::collect()
{
	//mavlink_log_info(&_mavlink_log_pub,"colect func");
	/* read from the sensor */
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int len=5; //was 4
	uint8_t val[len] {};
	int ret = transfer(nullptr, 0, &val[0], len);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		PX4_INFO("collect return ret < 0");
		return ret;
	}

	//uint8_t status = (val[0] & 0xC0) >> 6;
	//uint8_t status =transfer()
	// if((val[0] & 0x08)==0){
	// 	//empty values ==NAN
	// 	PRESSURE =0;
	// 	TEMPERATURE =0;
	// 	PX4_INFO("Empty values");
	// 	return ret;
	// }

	// switch (status) {
	// case 0:
	// 	// Normal Operation. Good Data Packet
	// 	break;

	// case 1:
	// 	// Reserved
	// 	return -EAGAIN;

	// case 2:
	// 	// Stale Data. Data has been fetched since last measurement cycle.
	// 	return -EAGAIN;

	// case 3:
	// 	// Fault Detected
	// 	perf_count(_comms_errors);
	// 	perf_end(_sample_perf);
	// 	return -EAGAIN;
	// }

	/* mask the used bits */
	//int16_t dp_raw = (0x3FFF & ((val[0] << 8) + val[1]));
	//int16_t dT_raw = (0xFFE0 & ((val[2] << 8) + val[3])) >> 5;

	//Pressure is a signed 24-bit value
	int k=7;
	double pressure_scala = 1.0 / (1<<k);
	PRESSURE=(((val[0]<< 24) | (val[1]<<16) | (val[2]<<8)) >> 8) *pressure_scala;


	//Temperature is a signed 16-bit value in units of 1/256 C
	TEMPERATURE = ((val[3]<<8) | val[4]);
	double temp_scala= 1.0/ 256;
	TEMPERATURE= TEMPERATURE * temp_scala;

	// press sum
	//press_sum += (PRESSURE * pressure_scala);
	//press_count++;

	//clock_t last_sample_time=clock();
	//returne PRESSURE, TEMPERATURE










	// dT max is almost certainly an invalid reading
	// if (dT_raw == 2047) {
	// 	perf_count(_comms_errors);
	// 	return -EAGAIN;
	// }

	// only publish changes
	if ((PRESSURE !=0 && TEMPERATURE !=0) && ((PRESSURE != PRESSURE_PREV) || (TEMPERATURE != TEMPERATURE_PREV))) {
	//if (1==1){
		//_dp_raw_prev = dp_raw;
		//_dT_raw_prev = dT_raw;
		PRESSURE_PREV =PRESSURE;
		TEMPERATURE_PREV=TEMPERATURE;

		//float temperature_c = ((200.0f * dT_raw) / 2047) - 50;

		// Calculate differential pressure. As its centered around 8000
		// and can go positive or negative
		const float P_min = -1.0f;
		const float P_max = 1.0f;
		const float IN_H20_to_Pa = 248.84f;
		/*
		  this equation is an inversion of the equation in the
		  pressure transfer function figure on page 4 of the datasheet

		  We negate the result so that positive differential pressures
		  are generated when the bottom port is used as the static
		  port on the pitot and top port is used as the dynamic port
		 */
		float diff_press_PSI = -((PRESSURE - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
		float diff_press_pa = diff_press_PSI * IN_H20_to_Pa;

		/*
		  With the above calculation the ASP5033 sensor will produce a
		  positive number when the top port is used as a dynamic port
		  and bottom port is used as the static port
		 */
		differential_pressure_s differential_pressure{};
		differential_pressure.timestamp_sample = timestamp_sample;
		differential_pressure.device_id = get_device_id();
		differential_pressure.differential_pressure_pa = diff_press_pa; //diff_press_pa
		differential_pressure.temperature = TEMPERATURE ; //temperature_c
		differential_pressure.error_count = perf_event_count(_comms_errors);
		differential_pressure.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(differential_pressure);
		_differential_pressure_sub.update(&_pressure); //param
	}

	perf_end(_sample_perf);

	return PX4_OK;
}
void ASP5033Driver::print_status()
{

	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);
}

void ASP5033Driver::RunImpl()
{
	int ret = PX4_ERROR;
	//mavlink_log_info(&_mavlink_log_pub,"RunImpl func");
	// Print result on console
	//PX4_INFO("Differential Pressure: %8.4f Pa ,Temperature: %8.4f C",
	//(double)PRESSURE,(double)TEMPERATURE);

	// collection phase
	if (_collect_phase) {
		// perform collection
		ret = collect();

		if (OK != ret) {
			perf_count(_comms_errors);
			/* restart the measurement state machine */
			_collect_phase = false;
			_sensor_ok = false;
			ScheduleNow();
			return;
		}

		// next phase is measurement
		_collect_phase = false;

		// is there a collect->measure gap?
		if (_measure_interval > CONVERSION_INTERVAL) {

			// schedule a fresh cycle call when we are ready to measure again
			ScheduleDelayed(_measure_interval - CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	ret = measure();


	// parameter_update_s update;

	// if (_parameter_update_sub.update(&update)) {
	// 	update_params();
	// }

	if (OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK);

	// next phase is collection
	_collect_phase = true;



	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
}




void ASP5033Driver::parameters_update()
{
	mavlink_log_info(&_mavlink_log_pub,"parameters_update func");
	if (_differential_pressure_sub.updated()) {
		differential_pressure_s differential_pressure_update;
		_differential_pressure_sub.copy(&differential_pressure_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}


void ASP5033Driver::update_params()
{
	updateParams();

	//_param_airspeed_scale[0] = _param_airspeed_scale_1.get();



}
