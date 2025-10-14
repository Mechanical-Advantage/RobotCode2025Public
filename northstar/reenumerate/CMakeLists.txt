cmake_minimum_required(VERSION 3.17)
project(renumerate)

add_executable(renumerate renumerate.c)
target_link_libraries(renumerate "-framework IOKit" "-framework CoreFoundation")
install(TARGETS renumerate DESTINATION bin)
