# driver source files
SET(sources Epos2.cpp epos2exceptions.cpp)
# application header files
SET(headers Epos2.h   epos2exceptions.h)

# locate the necessary dependencies
FIND_PACKAGE(iriutils)
FIND_PACKAGE(comm)
FIND_PACKAGE(LibFTDI1 NO_MODULE REQUIRED)

# add the necessary include directories
INCLUDE_DIRECTORIES(.)
  INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${comm_INCLUDE_DIR})
INCLUDE(${LIBFTDI_USE_FILE})

# create the shared library
ADD_LIBRARY(epos2_motor_controller SHARED ${sources})

# link necessary libraries
  TARGET_LINK_LIBRARIES(epos2_motor_controller ${iriutils_LIBRARY})
  TARGET_LINK_LIBRARIES(epos2_motor_controller ${comm_LIBRARY})
TARGET_LINK_LIBRARIES(epos2_motor_controller ${LIBFTDIPP_LIBRARIES})

INSTALL(TARGETS epos2_motor_controller
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/iridrivers
  ARCHIVE DESTINATION lib/iridrivers
)

INSTALL(FILES ${headers} DESTINATION include/iridrivers)

INSTALL(FILES ../Findepos2_motor_controller.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

ADD_SUBDIRECTORY(examples)

