
#include "ASP5033Driver.hpp"

ASP5033Driver::ASP5033Driver(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)

{
	_differential_pressure_pub.advertise();

}

ASP5033Driver::~ASP5033Driver()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);
}

int ASP5033Driver::probe()
{	//uint8_t cmd = 0;
	uint8_t cmd=REG_CMD_ASP5033;
	int ret = transfer(&cmd, 1, nullptr, 0);
	return ret;

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


float ASP5033Driver::get_differential_pressure()
{
	if(((double)(clock()-last_sample_time)/CLOCKS_PER_SEC)>0.1){
		return 0.0;
	}

	if(press_count == 0) {
		PRESSURE =PRESSURE_PREV;
        	return 0.0;
    	}

	//calc differential pressure
	PRESSURE= press_sum / press_count;
	PRESSURE_PREV=PRESSURE;
	press_sum=0.;
	press_count=0.;
	return PRESSURE;



}

int ASP5033Driver::measurment()
{
	perf_begin(_sample_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// //status check
	// uint8_t stat=REG_CMD_ASP5033;
	// transfer(&stat, 1, nullptr, 1);  //0=write,1=read //transfer(&cmd, 1, nullptr, 0)

	// uint8_t status;
	// int ret_status=transfer(nullptr, 0, &status, 1); // send,send_len, recieve,recieve_len

	// if (ret_status != PX4_OK) {
	// 	perf_count(_comms_errors);

	// }


	// if((status & 0x08)==0){
	// 	PRESSURE=0;
	// 	TEMPERATURE=0;

	// }
	if(1==4){
		PRESSURE=0;
	 	TEMPERATURE=0;

	}

	else{

		uint8_t cmd=REG_PRESS_DATA_ASP5033;
		int ret=transfer(&cmd, 1, nullptr, 0);
		if (ret != PX4_OK) {
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			//return ret;
			return -EAGAIN;
			//return PX4_ERROR;
		}
		// Read pressure and temperature as one block
		uint8_t val[5] {};
		ret = transfer(nullptr, 0, &val[0], sizeof(val));

		if (ret != PX4_OK) {
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			//return ret;
			return -EAGAIN;
			//return PX4_ERROR;
		}


		//Pressure is a signed 24-bit value
		int32_t press = (val[0]<<24) | (val[1]<<16) | (val[2]<<8);
		// convert back to 24 bit
		press >>= 8;
		// k is a shift based on the pressure range of the device. See
		// table in the datasheet
		constexpr uint8_t k = 7;
		constexpr float press_scale = 1.0 / (1U<<k);
		press_sum += press * press_scale;
		press_count++;
		// temperature is 16 bit signed in units of 1/256 C
		const int16_t temp = (val[3]<<8) | val[4];
		constexpr float temp_scale = 1.0 / 256;
		TEMPERATURE= temp *temp_scale;
		last_sample_time=clock();
		get_differential_pressure();

	}


	// update the publish values
	differential_pressure_s differential_pressure{};
	differential_pressure.timestamp_sample = timestamp_sample;
	differential_pressure.device_id = get_device_id();

	differential_pressure.differential_pressure_pa =(PRESSURE*0.001f);
	differential_pressure.temperature = TEMPERATURE ;

	//differential_pressure.differential_pressure_pa =(1000*0.001f);  //test values
	//differential_pressure.temperature = 33.11; //test values

	differential_pressure.error_count = perf_event_count(_comms_errors);
	differential_pressure.timestamp = hrt_absolute_time();
	_differential_pressure_pub.publish(differential_pressure);

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
	//measurment();
	//ScheduleDelayed(CONVERSION_INTERVAL);
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


int ASP5033Driver::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd_1 = CMD_MEASURE_ASP5033; //0;
	//uint8_t cmd_2 = REG_PRESS_DATA_ASP5033; //0;
	uint8_t cmd_3 = REG_CMD_ASP5033;; //0;
	//int ret_1 = transfer(&cmd_1, 1, nullptr, 0);
	//int ret_2 = transfer(&cmd_2, 1, nullptr, 0);
	//int ret_3 = transfer(&cmd_3, 1, nullptr, 0);

	//write
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(cmd_3);
	cmd[1] = static_cast<uint8_t>(cmd_1);
	int ret_4=transfer(&cmd[0], 2, nullptr, 0);



	if (OK != ret_4) {
		perf_count(_comms_errors);
	}

	return ret_4;
}

int ASP5033Driver::collect()
{

	perf_begin(_sample_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	if(1==4){
		PRESSURE=0;
	 	TEMPERATURE=0;

	}

	else{

		// uint8_t cmd=REG_PRESS_DATA_ASP5033;
		// int ret=transfer(&cmd, 1, nullptr, 0);
		// if (ret != PX4_OK) {
		// 	perf_count(_comms_errors);
		// 	perf_end(_sample_perf);
		// 	//return ret;
		// 	return -EAGAIN;
		// 	//return PX4_ERROR;
		// }
		// Read pressure and temperature as one block
		uint8_t val[5] {0,0,0,0,0};
		uint8_t cmd=REG_PRESS_DATA_ASP5033;
		transfer(&cmd, 1, &val[0], sizeof(val));
		// int ret_ = transfer(nullptr, 0, &val[0], sizeof(val));

		// if (ret_ != PX4_OK) {
		// 	perf_count(_comms_errors);
		// 	perf_end(_sample_perf);
		// 	//return ret;
		// 	return -EAGAIN;
		// 	//return PX4_ERROR;
		// }


		//Pressure is a signed 24-bit value
		int32_t press = (val[0]<<24) | (val[1]<<16) | (val[2]<<8);
		// convert back to 24 bit
		press >>= 8;
		// k is a shift based on the pressure range of the device. See
		// table in the datasheet
		constexpr uint8_t k = 7;
		constexpr float press_scale = 1.0 / (1U<<k);
		press_sum += press * press_scale;
		press_count++;
		// temperature is 16 bit signed in units of 1/256 C
		const int16_t temp = (val[3]<<8) | val[4];
		constexpr float temp_scale = 1.0 / 256;
		TEMPERATURE= temp *temp_scale;
		last_sample_time=clock();
		get_differential_pressure();

	}


	// update the publish values
	differential_pressure_s differential_pressure{};
	differential_pressure.timestamp_sample = timestamp_sample;
	differential_pressure.device_id = get_device_id();

	differential_pressure.differential_pressure_pa =PRESSURE; //(PRESSURE*0.001f);
	differential_pressure.temperature = TEMPERATURE ;

	//differential_pressure.differential_pressure_pa =(1000*0.001f);  //test values
	//differential_pressure.temperature = 33.11; //test values

	differential_pressure.error_count = perf_event_count(_comms_errors);
	differential_pressure.timestamp = hrt_absolute_time();
	_differential_pressure_pub.publish(differential_pressure);

	perf_end(_sample_perf);

	return PX4_OK;
}


