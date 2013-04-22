#
#	EXAMPLES	#
#
#
#	TEST EPOS2
ADD_EXECUTABLE(test_epos2 test_epos2.cpp)
#		dependences
TARGET_LINK_LIBRARIES(test_epos2 epos2_motor_controller)
#
#	CONFIGTEST
ADD_EXECUTABLE(configtest configtest.cpp)
#		dependences
TARGET_LINK_LIBRARIES(configtest epos2_motor_controller)
#
# TEST DELAYS
#ADD_EXECUTABLE(test_delays test_delays.cpp)
#   dependences
#TARGET_LINK_LIBRARIES(test_delays epos2_motor_controller)
#
# TEST like h3d
#ADD_EXECUTABLE(like_h3d test_like_h3d.cpp)
#   dependences
#TARGET_LINK_LIBRARIES(like_h3d epos2_motor_controller)
#
# TEST test positions
#ADD_EXECUTABLE(positions test_positions.cpp)
#   dependences
#TARGET_LINK_LIBRARIES(positions epos2_motor_controller)
#
# TEST test homing
ADD_EXECUTABLE(homing test_homing.cpp)
#   dependences
TARGET_LINK_LIBRARIES(homing epos2_motor_controller)
#
# TEST test marker
ADD_EXECUTABLE(marker test_marker.cpp)
#   dependences
TARGET_LINK_LIBRARIES(marker epos2_motor_controller)
