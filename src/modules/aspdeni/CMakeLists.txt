
px4_add_module(
	MODULE modules__aspdeni
	MAIN aspdeni
	SRCS
		aspdeni_main.cpp
		ASPDeni.cpp
		ASPDeni.hpp
	DEPENDS
		wind_estimator
)
