

/**
 *
 * Driver for ASP5033 connected via I2C.
 *
 * Supported sensors:
 *
 *    - ASP5033
 *
 * Interface application notes:
 *
 *
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>

#include <px4_platform_common/module.h> //deni
#include <lib/systemlib/mavlink_log.h>


static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface


/* Measurement rate is 100Hz */
#define MEAS_RATE 100 //100
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */


/* Configuration Constants */
#define ASP5033_BASEADDR         0x6D  // Adresse for communication to Pixhawk 0x6D

#define REG_CMD_ASP5033  0x30;
#define REG_PRESS_DATA_ASP5033 0X06
#define REG_TEMP_DATA_ASP5033 0X09
#define CMD_MEASURE_ASP5033 0X0A  //0x0A


class ASP5033Driver : public device::I2C , public I2CSPIDriver<ASP5033Driver>
{
public:
	ASP5033Driver(const I2CSPIDriverConfig &config);
	~ASP5033Driver() override;

	static void print_usage();
	void print_status() override;


	void RunImpl();

	int init() override;

	float PRESSURE = 0; //int16_t
	float TEMPERATURE = 0;
	float PRESSURE_PREV = 0;
	float TEMPERATURE_PREV = 0;

	float press_sum;
	uint32_t press_count;



	//test functions
	float differential_pressure_d(){
		return 333.00;
	}

	float temperature_d(){
		return 333.00;
	}




private:
	int probe() override;

	int measure();
	int collect();

	int measurment(); // r
	float get_differential_pressure(); //r
	clock_t last_sample_time=clock();

	orb_advert_t 	_mavlink_log_pub {nullptr}; //log send to

	uint32_t _measure_interval{CONVERSION_INTERVAL};
	uint32_t _conversion_interval{CONVERSION_INTERVAL};

	bool _sensor_ok{false};
	bool _collect_phase{false};

	uORB::PublicationMulti<differential_pressure_s> _differential_pressure_pub{ORB_ID(differential_pressure)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
	perf_counter_t _fault_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": fault detected")};
};

