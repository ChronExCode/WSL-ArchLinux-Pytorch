cmake_minimum_required(VERSION 3.20)
project(Robot VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-fdiagnostics-color=always)
endif()

# ── Dependencies ──────────────────────────────────────────────
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# ── Balance Controller Executable ─────────────────────────────
add_executable(robot
    src/main.cpp
    src/usb_manager.cpp
    src/odrive_usb.cpp
    src/msp_imu_usb.cpp
    src/balance_controller.cpp
    src/control_model.cpp
    src/ekf.cpp
    src/nmpc_controller.cpp
    src/tcp_server.cpp
    src/logger.cpp
    src/robot.cpp
)

target_include_directories(robot PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/../common
    ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(robot PRIVATE
    Threads::Threads
    ${LIBUSB_LIBRARIES}
)

target_compile_options(robot PRIVATE
    -Wall -Wextra -Wpedantic -O2 -march=native
)

set_target_properties(robot PROPERTIES OUTPUT_NAME balance_controller)


# Debug build enables libusb verbose logging
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(robot PRIVATE DEBUG)
endif()

install(TARGETS robot DESTINATION bin)
