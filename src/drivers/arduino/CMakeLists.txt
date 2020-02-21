############################################################################
px4_add_module(

	MODULE drivers__arduino
	MAIN arduino
	COMPILE_FLAGS
		-Wno-sign-compare
		-Wno-error
		-fpermissive
	SRCS
		arduino_i2c.cpp
	DEPENDS
	)