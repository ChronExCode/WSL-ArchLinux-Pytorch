cmake_minimum_required(VERSION 3.20)
project(RobotController VERSION 1.0.0 LANGUAGES C CXX)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-fdiagnostics-color=always)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Dependencies ──────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# Wayland
pkg_check_modules(WAYLAND_CLIENT REQUIRED wayland-client)
pkg_check_modules(XKB REQUIRED xkbcommon)

# Cairo for rendering (Wayland buffer painting)
pkg_check_modules(CAIRO REQUIRED cairo)

# ── Wayland protocol scanner ─────────────────────────────────
find_program(WAYLAND_SCANNER wayland-scanner REQUIRED)

set(WL_PROTOCOLS_DIR "/usr/share/wayland-protocols")

# Generate xdg-shell protocol
set(XDG_SHELL_XML "${WL_PROTOCOLS_DIR}/stable/xdg-shell/xdg-shell.xml")
set(XDG_SHELL_H "${CMAKE_BINARY_DIR}/xdg-shell-protocol.h")
set(XDG_SHELL_C "${CMAKE_BINARY_DIR}/xdg-shell-protocol.c")

add_custom_command(
    OUTPUT ${XDG_SHELL_H}
    COMMAND ${WAYLAND_SCANNER} client-header ${XDG_SHELL_XML} ${XDG_SHELL_H}
    DEPENDS ${XDG_SHELL_XML}
)
add_custom_command(
    OUTPUT ${XDG_SHELL_C}
    COMMAND ${WAYLAND_SCANNER} private-code ${XDG_SHELL_XML} ${XDG_SHELL_C}
    DEPENDS ${XDG_SHELL_XML}
)

# ── Controller Executable ────────────────────────────────────
add_executable(controller
    src/main.cpp
    src/hud_renderer.cpp
    src/config_persistence.cpp
    ${XDG_SHELL_C}
    ${XDG_SHELL_H}
)

target_include_directories(controller PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/../common
    ${CMAKE_BINARY_DIR}
    ${WAYLAND_CLIENT_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${XKB_INCLUDE_DIRS}
)

target_link_libraries(controller PRIVATE
    Threads::Threads
    ${WAYLAND_CLIENT_LIBRARIES}
    ${CAIRO_LIBRARIES}
    ${XKB_LIBRARIES}
    rt
)

target_compile_options(controller PRIVATE -Wall -Wextra -O2)

set_target_properties(controller PROPERTIES OUTPUT_NAME controller)

install(TARGETS controller DESTINATION bin)
