
#include "ASP5033Driver.hpp"


#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/differential_pressure.h>
#include <uORB/Publication.hpp> //publush
#include <drivers/drv_hrt.h> //time



#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>




class ASP5033Driver : public ModuleBase<ASP5033Driver>
{
	public:
	// default constructor
	Asp5033Driver();
	// metotods requiret by modulebase inheritance


	void Run() override;

	uORB::Publication<differential_pressure_s>  _diff_press_pub{ORB_ID(differential_pressure)};
       	differential_pressure_s _diff_press{}; // This is the uORB ‘safety’ message definition struct
	hrt_abstime _time_now_usec_d{0};


	void Run()
	{
	orb_advert_t asp5033_pub = orb_advertise(ORB_ID(differential_pressure), &_diff_press)
	_diff_press.timestamp = _time_now_usec_d;
	_diff_press.differential_pressure_pa=differential_pressure_d();
	_diff_press.temperature=temperature_d();
	_diff_press_pub.publish(_diff_press);
	//orb_publish(ORB_ID(differential_pressure), asp5033_pub, &_diff_press);
	}



};


// extern "C" int asp5033driver_main(int argc, char *argv[])
// {
// 	//return ASP5033::Run();
// 	return ASP5033Driver::main(argc, argv);

// 	//ASP5033 asp;
// 	//asp.Run();
// 	//return -1;
// 	// using ThisDriver = ASP5033;
// 	// BusCLIArguments cli{true, false};
// 	// cli.i2c_address = I2C_ADDRESS_DEFAULT;
// 	// cli.default_i2c_frequency = I2C_SPEED;

// 	// const char *verb = cli.parseDefaultArguments(argc, argv);

// 	// if (!verb) {
// 	// 	ThisDriver::print_usage();
// 	// 	return -1;
// 	// }
// 	// ThisDriver::value();// added
// 	// ASP5033 asp;
// 	// asp.value();

// 	// BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIFF_PRESS_DEVTYPE_MS4515);



// }
