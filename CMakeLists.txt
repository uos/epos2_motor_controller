cmake_minimum_required(VERSION 3.8)
project(epos2_motor_controller VERSION 1.0.3)

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

add_library(epos2 src/Epos2.cpp)
target_link_libraries(epos2
  ${FTDI_LIBRARIES}
)

target_include_directories(epos2 PUBLIC ${FTDI_INCLUDE_DIRS})
target_include_directories(epos2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Install includes
install(
  DIRECTORY include/
  DESTINATION include
)

# Install lib 
install(
  TARGETS epos2
  EXPORT epos2Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  # Export external dependency to libftdi to make sure that
  # packages that depend on this are able to find our custom
  # cmake script for libftdi in their cmake scripts
  ament_export_dependencies(FTDI)

  # Signal that the package build its own library
  ament_export_targets(epos2Targets HAS_LIBRARY_TARGET)

  # Package everything including our custom cmake script
  ament_package(CONFIG_EXTRAS cmake/FindFTDI.cmake)
endif(ament_cmake_FOUND)