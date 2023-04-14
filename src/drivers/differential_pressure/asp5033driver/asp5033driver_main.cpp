

#include "ASP5033Driver.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
//#define I2C_ADDRESS_ASP5033 0x6D




void ASP5033Driver::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description: This ASP5033 driver-differential pressure module is
 integrated by Denis,Test Version)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("asp5033driver", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("airspeed_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(ASP5033_BASEADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int asp5033driver_main(int argc, char *argv[])
{
	PX4_INFO("DENISLAV-3");
	//mavlink_log_info(&_mavlink_log_pub,"enter main function");

	using ThisDriver = ASP5033Driver;
	BusCLIArguments cli{true, false};
	cli.i2c_address = ASP5033_BASEADDR;
	cli.default_i2c_frequency = I2C_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);



	if (!verb) {
		//PX4_INFO("return -1");
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIFF_PRESS_DEVTYPE_ASP5033);

	//ThisDriver::print_status(cli,iterator); //DEBUG

	if (!strcmp(verb, "start")) {
		//PX4_INFO("enter start");
		//mavlink_log_info(&_mavlink_log_pub,"enter start-mav");
		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
