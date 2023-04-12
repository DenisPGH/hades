

/**
 * @file deni_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>


#include "deni.hpp"
#include <uORB/topics/differential_pressure.h>
#include <drivers/differential_pressure/asp5033driver/ASP5033Driver.hpp>

//make
// PATHS
//  /home/nano/Firmware/src/modules/mavlink/mavlink_messages.cpp
//  /home/nano/Firmware/src/modules/mavlink/streams/VFR_HUD.hpp
//  /home/nano/Firmware/build/px4_sitl_default/mavlink/common/mavlink_msg_vfr_hud.h
//  /home/nano/Firmware/platforms/common/uORB/Subscription.hpp
//#include <platforms/common/uORB/uORB.h>
// /home/nano/Firmware/platforms/common/uORB/Publication.hpp
//#include <msg/topics_sources/uORBTopics.cpp>
//  /home/nano/Firmware/build/px4_sitl_default/msg/topics_sources/airspeed.cpp
//  /home/nano/Firmware/msg/Airspeed_ASP503.msg
//#include <modules/mavlink/streams/VFR_HUD.hpp>
///  /home/nano/Firmware/msg/CMakeLists.txt
// /home/nano/Firmware/build/px4_sitl_default/src/examples/deni
// /home/nano/Firmware/build/px4_sitl_default/compile_commands.json
// /home/nano/Firmware/build/px4_sitl_default/platforms/posix/apps.cpp

// /home/nano/Firmware/build/px4_sitl_default/uORB/topics/differential_pressure.h
// #include <modules/mavlink/streams/SCALED_PRESSURE3.hpp>
//  /home/nano/Firmware/platforms/common/include/px4_platform_common/module.h
//  /home/nano/Firmware/ROMFS/px4fmu_common/init.d/rcS
// /home/nano/Firmware/ROMFS/px4fmu_common/init.d-posix/rcS
// /home/nano/Firmware/ROMFS/cannode/init.d/rcS
// /home/nano/Firmware/posix-configs/SITL/init/test/test_cmd_template.in
// /home/nano/Firmware/posix-configs/SITL/init/test/test_mavlink


// /home/nano/Firmware/ROMFS/px4fmu_common/init.d/rcS == this file run my script in the PC
// /home/nano/Firmware/boards/px4/fmu-v6c/default.px4board need this for Pixhawk 6c

// /home/nano/Firmware/Tools/upload.sh --> start upload
// asp5033driver #0 on I2C bus 4 address 0x6D
// /home/nano/Firmware/src/drivers/drv_sensor.h



class Air: public ASP5033Driver {
	public:
	int main(int argc, char *argv[]){


		struct differential_pressure_s asp5033_;

		//PX4_INFO("before temp: %8.4f ms",(double)asp5033_.temperature);
		//PX4_INFO(" press: %8.4f ms",(double)asp5033_.differential_pressure_pa);

		memset(&asp5033_, 0, sizeof(asp5033_));
		orb_advert_t asp5033_pub = orb_advertise(ORB_ID(differential_pressure), &asp5033_);
		//orb_copy(ORB_ID(airspeed__a_s_p5033), asp5033_pub, &get_temp());
		//while (){
		asp5033_.temperature=temperature_d();
		asp5033_.differential_pressure_pa=differential_pressure_d();
		orb_publish(ORB_ID(differential_pressure), asp5033_pub, &asp5033_);

		//}

		PX4_INFO("ASP5033 temp: %8.4f ms",(double)asp5033_.temperature);
		PX4_INFO("ASP5033 press: %8.4f ms",(double)asp5033_.differential_pressure_pa);



		return 0;

	}


};







/* from this extern part got the main program and cant print*/
extern "C" __EXPORT int deni_main(int argc, char *argv[])
{
	Air fk;
	return fk.main(argc, argv);
}


