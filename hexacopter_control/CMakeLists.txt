cmake_minimum_required(VERSION 3.10)
project(HexacopterControl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ArduPilot includes
set(ARDUPILOT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ardupilot")
if(NOT EXISTS ${ARDUPILOT_ROOT})
    message(FATAL_ERROR "ArduPilot not found at ${ARDUPILOT_ROOT}")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ARDUPILOT_ROOT}/libraries
    ${ARDUPILOT_ROOT}/libraries/AP_Common
    ${ARDUPILOT_ROOT}/libraries/AP_Math
    ${ARDUPILOT_ROOT}/libraries/AP_HAL
    ${ARDUPILOT_ROOT}/libraries/AP_HAL_Empty
    ${ARDUPILOT_ROOT}/libraries/AP_Param
    ${ARDUPILOT_ROOT}/libraries/AP_Motors
    ${ARDUPILOT_ROOT}/libraries/AP_Logger
    ${ARDUPILOT_ROOT}/libraries/StorageManager
)

# Source files
set(SOURCES
    src/hexacopter_sysid.cpp
    src/ekf_attitude_velocity.cpp
    src/ekf_attitude_velocity_adaptive.cpp
    src/velocity_feedback_controller.cpp
    src/unified_estimator.cpp
)

# Create library
add_library(hexacopter_control STATIC ${SOURCES})

# Compiler flags
target_compile_options(hexacopter_control PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-missing-field-initializers
)

# Define HAL_BOARD for SITL
target_compile_definitions(hexacopter_control PRIVATE
    HAL_BOARD=HAL_BOARD_SITL
    HAL_BOARD_SITL=13
    HAL_LOGGING_ENABLED=1
)

# Test executable (standalone)
add_executable(test_build tests/test_build.cpp)
target_link_libraries(test_build hexacopter_control)