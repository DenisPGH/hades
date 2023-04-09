

#include "ASP5033Driver.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>




void ASP5033Driver::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ASP5033Driver", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("airspeed_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(I2C_ADDRESS_DEFAULT);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int ASP5033Driver_main(int argc, char *argv[])
{
	using ThisDriver = ASP5033Driver;
	BusCLIArguments cli{true, false};
	cli.i2c_address = I2C_ADDRESS_ASP5033;
	cli.default_i2c_frequency = I2C_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIFF_PRESS_DEVTYPE_MS4515);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
