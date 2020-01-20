cmake_minimum_required(VERSION 2.8)
project(libpru)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(libsrc pru.c ti/ti-pru.c ti/am335x/ti-am335x-pru.c)
add_definitions(-I${CMAKE_SOURCE_DIR} -I${CMAKE_SOURCE_DIR}/ti -I${CMAKE_SOURCE_DIR}/ti/am335x)

add_library(pru SHARED ${libsrc})
add_compile_definitions(WITH_KQUEUE=1)
set_property(TARGET pru PROPERTY SOVERSION "3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror -fblocks")
target_link_libraries(pru pthread BlocksRuntime)
install(TARGETS pru DESTINATION lib)
install(FILES libpru.h DESTINATION include)

add_executable(pructl pructl.c ${libsrc} )

add_executable(prudbg pructl.c ${libsrc} )
find_library(libedit NAMES edit)
find_library(libutil NAMES util)
target_link_libraries(prudbg ${libedit} ${libutil})
install(TARGETS pructl prudbg DESTINATION sbin)

