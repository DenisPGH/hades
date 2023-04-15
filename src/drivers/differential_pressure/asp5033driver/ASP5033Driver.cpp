

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
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, nullptr, 0);
	return ret;
	//return PX4_OK;

}

int ASP5033Driver::init()
{
	int ret = I2C::init();
	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		DEVICE_DEBUG("I2C::init successed (%i)", ret);
		ScheduleNow();
	}

	return ret;
}

int ASP5033Driver::measure()
{
	// Send the command to begin a measurement.

	//uint8_t cmd=REG_CMD_ASP5033; // no error in baro- not start the AP
	uint8_t cmd=REG_PRESS_DATA_ASP5033; // one time run and then stop==best variant
	//uint8_t cmd= CMD_MEASURE_ASP5033; //error was in baro==barometer 0 missing
	//uint8_t cmd=0; //default
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int ASP5033Driver::collect()
{
	perf_begin(_sample_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	//Request a new measurment cycle
	//uint8_t cmd_status=REG_CMD_ASP5033;
	//uint8_t status = transfer(&cmd_status, 1, nullptr, 0);

	//if((status & 0x08)==0){
	//	return -EAGAIN;
	//}

	// Read pressure and temperature as one block
	//uint8_t read_data=REG_PRESS_DATA_ASP5033;
	uint8_t val[5] {};
	//int ret = transfer(nullptr, 0, &val[0], sizeof(val));
	transfer(nullptr, 0, val, sizeof(val));
	// if (ret < 0) {
	// 	perf_count(_comms_errors);
	// 	perf_end(_sample_perf);
	// 	PX4_INFO("collect return ret < 0");
	// 	return ret;
	// }
	//PX4_INFO("WORK GOOD, Denis!!");

	// if((val[0] & 0x08)==0){
	// 	return -EAGAIN;
	// }

	//Pressure is a signed 24-bit value
    	int32_t press = (val[0]<<24) | (val[1]<<16) | (val[2]<<8);
    	// convert back to 24 bit
    	press >>= 8;

	// k is a shift based on the pressure range of the device. See
    	// table in the datasheet
    	constexpr uint8_t k = 7;
    	constexpr float press_scale = 1.0 / (1U<<k);

	//constexpr uint8_t k=7;
	//constexpr float press_scale = 1.0 / (1U<<k);
	//PRESSURE=(((val[0]<< 24) | (val[1]<<16) | (val[2]<<8)) >> 8) *press_scale;
	press_sum += press * press_scale;
	//press_sum += PRESSURE;
	press_count++;


	// temperature is 16 bit signed in units of 1/256 C
    	const int16_t temp = (val[3]<<8) | val[4];
    	constexpr float temp_scale = 1.0 / 256;

	/// temperature is 16 bit signed in units of 1/256 C
    	//const int16_t temp = (val[3]<<8) | val[4];
    	//constexpr float temp_scale = 1.0 / 256;
	TEMPERATURE= temp *temp_scale;

	clock_t last_sample_time=clock();

	// ========================================

	//if(clock()-last_sample_time>100){
	if(((double)(clock()-last_sample_time)/CLOCKS_PER_SEC)>0.1){
		return -EAGAIN;
	}

	if(press_count == 0) {
		PRESSURE =PRESSURE_PREV;
        	return -EAGAIN;
    	}

	//calc differential pressure
	PRESSURE_PREV=PRESSURE= press_sum / press_count;
	press_sum=0.0;
	press_count=0.0;



	// update the publish values
	differential_pressure_s differential_pressure{};
	differential_pressure.timestamp_sample = timestamp_sample;
	differential_pressure.device_id = get_device_id();
	differential_pressure.differential_pressure_pa = (PRESSURE_PREV*0.001f); //diff_press_pa
	differential_pressure.temperature = TEMPERATURE ; //temperature_c
	differential_pressure.error_count = perf_event_count(_comms_errors);
	differential_pressure.timestamp = hrt_absolute_time();
	_differential_pressure_pub.publish(differential_pressure);
	_differential_pressure_sub.update(&_pressure); //param



	//change with time step
	// only publish changes
	//if ((PRESSURE !=0 && TEMPERATURE !=0) && ((PRESSURE != PRESSURE_PREV) || (TEMPERATURE != TEMPERATURE_PREV))) {
	// if (1==1){
	// 	PRESSURE_PREV =PRESSURE;
	// 	TEMPERATURE_PREV=TEMPERATURE;
	// 	const float P_min = -1.0f;
	// 	const float P_max = 1.0f;
	// 	const float IN_H20_to_Pa = 248.84f;
	// 	float diff_press_PSI = -((PRESSURE - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
	// 	float diff_press_pa = diff_press_PSI * IN_H20_to_Pa;
	// 	differential_pressure_s differential_pressure{};
	// 	differential_pressure.timestamp_sample = timestamp_sample;
	// 	differential_pressure.device_id = get_device_id();
	// 	differential_pressure.differential_pressure_pa = (diff_press_pa/1000); //diff_press_pa
	// 	differential_pressure.temperature = TEMPERATURE ; //temperature_c
	// 	differential_pressure.error_count = perf_event_count(_comms_errors);
	// 	differential_pressure.timestamp = hrt_absolute_time();
	// 	_differential_pressure_pub.publish(differential_pressure);
	// 	_differential_pressure_sub.update(&_pressure); //param
	// }

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
	//_asp5033_parameter=1;



}
