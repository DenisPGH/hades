
px4_add_module(
	MODULE modules__asp5033
	MAIN asp5033
	SRCS
		asp5033_main.cpp
		ASP5033.cpp
		ASP5033.hpp
	DEPENDS
		px4_work_queue

)
