#edit the following line to add the librarie's header files
FIND_PATH(epos2_motor_controller_INCLUDE_DIR Epos2.h epos2exceptions.h /usr/include/iridrivers /usr/local/include/iridrivers)

FIND_LIBRARY(epos2_motor_controller_LIBRARY
    NAMES epos2_motor_controller
    PATHS /usr/lib/iridrivers /usr/local/lib /usr/local/lib/iridrivers) 

IF (epos2_motor_controller_INCLUDE_DIR AND epos2_motor_controller_LIBRARY)
   SET(epos2_motor_controller_FOUND TRUE)
ENDIF (epos2_motor_controller_INCLUDE_DIR AND epos2_motor_controller_LIBRARY)

IF (epos2_motor_controller_FOUND)
   IF (NOT epos2_motor_controller_FIND_QUIETLY)
      MESSAGE(STATUS "Found epos2_motor_controller: ${epos2_motor_controller_LIBRARY}")
   ENDIF (NOT epos2_motor_controller_FIND_QUIETLY)
ELSE (epos2_motor_controller_FOUND)
   IF (epos2_motor_controller_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find epos2_motor_controller")
   ENDIF (epos2_motor_controller_FIND_REQUIRED)
ENDIF (epos2_motor_controller_FOUND)

