cmake_minimum_required(VERSION 3.12)
project(mdfly VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external
)

# Source files
set(LIBRARY_SOURCES
    src/variable.cpp
    src/attitude.cpp
    src/wind.cpp
    src/actuation.cpp
    src/simulator.cpp
    src/pid_controller.cpp
    src/motor_model.cpp
    src/camera_model.cpp
    src/mavlink_comm.cpp
)

# Header files (for IDE visualization)
set(LIBRARY_HEADERS
    include/mdfly/variable.h
    include/mdfly/attitude.h
    include/mdfly/wind.h
    include/mdfly/actuation.h
    include/mdfly/simulator.h
    include/mdfly/pid_controller.h
    include/mdfly/motor_model.h
    include/mdfly/camera_model.h
    include/mdfly/mavlink_comm.h
)

# Create library
add_library(mdfly STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})

# Create main executable
add_executable(mdfly_example examples/main.cpp)
target_link_libraries(mdfly_example PRIVATE mdfly)

# Scenario 1 Executable
add_executable(mdfly_scenario1 examples/scenario1_formation.cpp)
target_link_libraries(mdfly_scenario1 PRIVATE mdfly)

# Scenario 2 Executable
add_executable(mdfly_scenario2 examples/scenario2_tracking.cpp)
target_link_libraries(mdfly_scenario2 PRIVATE mdfly)

# Scenario 3 Executable
add_executable(mdfly_scenario3 examples/scenario3_takeoff_landing.cpp)
target_link_libraries(mdfly_scenario3 PRIVATE mdfly)

# Scenario 4 Executable
add_executable(mdfly_scenario4 examples/scenario4_qgc.cpp)
target_link_libraries(mdfly_scenario4 PRIVATE mdfly)
