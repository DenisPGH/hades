
#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>

class Fake {

};
// class FakeAirspeeD{
// 	public:
// 	bfs::Ms4525do airsensor;
// 	FakeAirSensor fakesensor;
// 	double air_rate=3.345;
// 	double Wire=333.33;
// 	float a=0.0;

// 	double get_temp(){
// 		airsensor.Config(&Wire, 0x28, 1.0f, -1.0f);
// 		if (! airsensor.Begin()){
// 			a=0.0;
// 		}
// 		else if (airsensor.Read()) {
//     			a=airsensor.die_temp_c();

//   		}
// 		return a;

// 	}

// 	double get_air_sensor(){
// 		return fakesensor.return_sensor_value();
// 	}

// 	int get_test_inher(){
// 		return fakesensor.test_inher();
// 	}




// };
