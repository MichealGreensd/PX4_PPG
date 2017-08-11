px4_add_module(
	MODULE sensor__airsensor
	MAIN airsensor
	STACK_MAIN 1200
	SRCS
		airsensor.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 