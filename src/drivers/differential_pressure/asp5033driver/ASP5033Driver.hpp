

// #include <random>
// #include <iostream>
// #include <ctime>


// //import time
// //import smbus
// #define I2C_ADDRESS 0x6D
// #define I2C_BUS 1

// #define REG_CMD 0x30
// #define REG_PRESS_DATA 0X06
// #define REG_TEMP_DATA 0X09
// #define CMD_MEASURE 0X0A




// class ASP5033Driver
// {
// 	public:
// 	float TEMPERATURE=NAN;
// 	float PRESSURE=NAN;

// 	int press_sum=0;
// 	int press_count=0;
// 	const clock_t last_sample_time = clock();

// 	void empty_values(){
// 		TEMPERATURE=NAN;
// 		PRESSURE=NAN;

// 	}
// 	// void read_sensor_data(auto bus){
// 	// 	try
// 	// 	{
// 	// 		//Request a new measurment cycle
// 	// 		bus.write_byte_data(I2C_ADDRESS,REG_CMD,CMD_MEASURE);
// 	// 		auto status= bus.read_byte_data(I2C_ADDRESS,REG_CMD);
// 	// 		if(status & 0x08)==0{
// 	// 			empty_values();

// 	// 		}
// 	// 		// Read pressure and temperature as one block
// 	// 		data=bus.read_i2c_block_data(I2C_ADDRESS,REG_PRESS_DATA,5);


// 	// 	}
// 	// 	catch(const std::exception& e)
// 	// 	{
// 	// 		empty_values();
// 	// 		PX4_INFO("Error");

// 	// 	}
// 	// 	//Pressure is a signed 24-bit value
// 	// 	PRESSURE=(data[0]<< 24) | (data[1]<<16) | (data[2]<<8)
// 	// 	PRESSURE= PRESSURE >> 8;
// 	// 	int k=7;
// 	// 	double pressure_scala = 1.0/ (1<<k);

// 	// 	//Temperature is a signed 16-bit value in units of 1/256 C
// 	// 	TEMPERATURE = ((data[3]<<8) | data[4]);
// 	// 	double temp_scala= 1.0/ 256;
// 	// 	TEMPERATURE= TEMPERATURE * temp_scala;

// 	// 	// press sum
// 	// 	press_sum += (PRESSURE * pressure_scala);
// 	// 	press_count++;

// 	// 	clock_t last_sample_time=clock();
// 	// 	//returne PRESSURE, TEMPERATURE

// 	// }

// 	// float get_differential_pressure(){
// 	// 	if(clock()- last_sample_time) >0.1{
// 	// 		return NAN;
// 	// 	}
// 	// 	if (press_count==0){
// 	// 		return NAN;
// 	// 	}
// 	// 	float final_differential_pressure= press_sum / press_count;
// 	// 	press_count=0.0;
// 	// 	press_sum=0.0;
// 	// 	return final_differential_pressure;

// 	// }

// 	// void main(){
// 	// 	bus=smbus.SMBus(I2C_BUS);
// 	// 	//while(true)
// 	// 	read_sensor_data(bus); // read the values
// 	// 	if ((TEMPERATURE != NAN) && (PRESSURE != NAN)){
// 	// 		float diff_press= get_differential_pressure();
// 	// 		if (diff_press != NAN){
// 	// 			PX4_INFO("Differential Pressure: %8.4f Pa ,Temperature: %8.4f C",
// 	// 			(float)PRESSURE,
// 	// 			(float)TEMPERATURE);

// 	// 		}
// 	// 	}


// 	// }

// 	float differential_pressure_d(){
// 		std::random_device dev;
//     		std::mt19937 rng(dev());
//     		std::uniform_int_distribution<std::mt19937::result_type> dist6(1000,1020);

// 		return (float)dist6(rng);
// 	}

// 	float temperature_d(){
// 		std::random_device dev_2;
//     		std::mt19937 rng(dev_2());
//     		std::uniform_int_distribution<std::mt19937::result_type> dist6(27,32);

// 		return (float)dist6(rng);
// 	}



// };






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
 *    - link to source dokumentation
 */

#pragma once
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
//#include <lib/drivers/device/posix/I2C.hpp>

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>

static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x6D; //adress of ASp5033

/* Register address */
#define ADDR_READ_MR			0x00	/* write to this address to start conversion */

/* Measurement rate is 100Hz */
#define MEAS_RATE 100
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */

class ASP5033Driver : public device::I2C, public I2CSPIDriver<ASP5033Driver>
{
public:
	ASP5033Driver(const I2CSPIDriverConfig &config);
	~ASP5033Driver() override;

	static void print_usage();

	void RunImpl();

	int init() override;

	float differential_pressure_d(){
		std::random_device dev;
    		std::mt19937 rng(dev());
    		std::uniform_int_distribution<std::mt19937::result_type> dist6(1000,1020);

		return (float)dist6(rng);
	}

	float temperature_d(){
		std::random_device dev_2;
    		std::mt19937 rng(dev_2());
    		std::uniform_int_distribution<std::mt19937::result_type> dist6(27,32);

		return (float)dist6(rng);
	}



private:
	int probe() override;

	int measure();
	int collect();

	uint32_t _measure_interval{CONVERSION_INTERVAL};
	uint32_t _conversion_interval{CONVERSION_INTERVAL};

	int16_t _dp_raw_prev{0};
	int16_t _dT_raw_prev{0};

	bool _sensor_ok{false};
	bool _collect_phase{false};

	uORB::PublicationMulti<differential_pressure_s> _differential_pressure_pub{ORB_ID(differential_pressure)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
};

