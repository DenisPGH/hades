

#include <random>
#include <iostream>



class ASP5033Driver
{
	public:

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
};
