# This is used to test the installation process

cmake_minimum_required(VERSION 3.10)
project(libusb_demo)

set(CMAKE_C_STANDARD 99) # LVGL officially supports C99 and above
set(CMAKE_CXX_STANDARD 17) #C17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Uncomment if the program needs debugging
set(CMAKE_BUILD_TYPE Debug)

find_package(PkgConfig REQUIRED)

#find_package(libusb-1.0 REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)


#pkg_check_modules(LVGL REQUIRED lvgl)

#set(SCSERVO_ROOT "SCServo_Linux")

#link_directories(./SCServo_Linux)
#link_libraries(libSCServo.a)

#add_executable(main main.cpp lv_gui.c) 
add_executable(libusb_demo main.cpp)  

#find_library(gyro_control PUBLIC ${SCSERVO_ROOT})

target_include_directories(libusb_demo PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_directories   (libusb_demo PRIVATE ${LIBUSB_LIBRARY_DIRS})
target_link_libraries     (libusb_demo PRIVATE ${LIBUSB_LIBRARIES})


#target_link_libraries(gyro_control PUBLIC lvgl_linux ${LVGL_LIBRARIES}  lvgl_linux m glfw stdc++)

#add_custom_target (run COMMAND ${EXECUTABLE_OUTPUT_PATH}/main DEPENDS main)
