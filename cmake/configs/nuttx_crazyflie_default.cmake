include(nuttx/px4_impl_nuttx)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/stm32
	drivers/led
	drivers/boards/crazyflie
	# drivers/mpu9250
	drivers/gps
	# drivers/pwm_out_sim
	# modules/sensors

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/mtd
	systemcmds/dumpfile
	systemcmds/ver

	#
	# General system control
	#
	modules/commander
	modules/navigator
	modules/mavlink
	modules/land_detector

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	# Too high RAM usage due to static allocations
	# modules/attitude_estimator_ekf
	modules/attitude_estimator_q
	modules/ekf_att_pos_estimator
	modules/position_estimator_inav

	#
	# Vehicle Control
	#
	# modules/segway # XXX Needs GCC 4.7 fix
	modules/fw_pos_control_l1
	modules/fw_att_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	modules/sdlog2

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/controllib
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	#lib/mathlib/CMSIS
	lib/mathlib
	lib/mathlib/math/filter
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/launchdetection
	lib/terrain_estimation
	lib/runway_takeoff
	platforms/nuttx

	# had to add for cmake, not sure why wasn't in original config
	platforms/common 
	platforms/nuttx/px4_layer

	#
	# OBC challenge
	#
	modules/bottle_drop

	#
	# Rover apps
	#
	examples/rover_steering_control

	#
	# Demo apps
	#
	#examples/math_demo
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	#examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	#examples/px4_daemon_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	#examples/hwtest
)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

set(config_extra_libs
	${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM4lf_math.a
	)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	MAIN "sercon" STACK "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	MAIN "serdis" STACK "2048")
