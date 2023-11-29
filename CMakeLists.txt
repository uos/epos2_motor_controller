cmake_minimum_required(VERSION 3.2)
project(epos2_motor_controller VERSION 1.0.2)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

set(CMAKE_MODULE_PATH
  ${PROJECT_SOURCE_DIR}/cmake
  ${CMAKE_MODULE_PATH}
)

# FIND PACKAGE FTDI
# - FTDI_FOUND
# - FTDI_INCLUDE_DIRS
# - FTDI_LIBRARIES
find_package(FTDI REQUIRED)
include_directories(${FTDI_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES epos2
  CATKIN_DEPENDS roscpp
  DEPENDS FTDI
)

include_directories(include)

add_library(epos2 src/Epos2.cpp)
target_link_libraries(epos2
  ${FTDI_LIBRARIES}
)

### INSTALL ###
install(TARGETS epos2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)