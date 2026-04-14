cmake_minimum_required(VERSION 3.20)
project(SelfBalanceRobot VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Dependencies ──────────────────────────────────────────────
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# ── Balance Controller Executable ─────────────────────────────
add_executable(balance_controller
    src/main.cpp
    src/usb_manager.cpp
    src/odrive_usb.cpp
    src/msp_imu_usb.cpp
    src/balance_controller.cpp
    src/tcp_server.cpp
    src/robot.cpp
)

target_include_directories(balance_controller PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(balance_controller PRIVATE
    Threads::Threads
    ${LIBUSB_LIBRARIES}
)

target_compile_options(balance_controller PRIVATE
    -Wall -Wextra -O2 -march=native
)

# Debug build enables libusb verbose logging
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(balance_controller PRIVATE DEBUG)
endif()

install(TARGETS balance_controller DESTINATION bin)
