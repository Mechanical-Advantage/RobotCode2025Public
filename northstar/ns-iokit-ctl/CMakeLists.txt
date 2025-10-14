cmake_minimum_required(VERSION 3.1)
project(ns_iokit_ctl)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(openpnp-capture)

add_executable(ns_iokit_ctl main.cpp)
target_link_libraries(ns_iokit_ctl openpnp-capture)
