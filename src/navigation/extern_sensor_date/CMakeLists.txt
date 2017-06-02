
px4_add_module(
	MODULE navigation__extern_sensor_date
	MAIN extern_sensor_date
	STACK_MAIN 2000
	SRCS
		extern_sensor_date.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
