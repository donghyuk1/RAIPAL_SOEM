#cmake_minimum_required(VERSION 3.30)
#project(soem_example)
#
#set(CMAKE_CXX_STANDARD 20)
#set(RAISIN_ROOT "$ENV{HOME}/Desktop/raisin_ws/raisin_master")
#set(RAISIN_BUILD "$ENV{HOME}/Desktop/raisin_ws/raisin_master/build")
#set(SOEM_LIB "${RAISIN_BUILD}/src/raisin/soem/libsoem.a")
#
#add_library(soem STATIC IMPORTED GLOBAL)
#set_target_properties(soem PROPERTIES
#        IMPORTED_LOCATION "${SOEM_LIB}"
#        INTERFACE_INCLUDE_DIRECTORIES
#        "${RAISIN_ROOT}/src/raisin/soem/soem;${RAISIN_ROOT}/src/raisin/soem/osal;${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
#)
#
## Add path to SOEM's cmake config
#list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Desktop/SOEM-install/cmake")
#
#add_executable(simple_test simple_test.c)
#
## SOEM headers inside raisin (so ethercat.h resolves)
## SOEM headers (add the linux OSAL folder!)
#target_include_directories(simple_test PRIVATE
#        "${RAISIN_ROOT}/src/raisin/soem/soem"        # ethercat.h, etc.
#        "${RAISIN_ROOT}/src/raisin/soem/osal"
#        "${RAISIN_ROOT}/src/raisin/soem/osal/linux"  # <— THIS fixes osal_defs.h
#        "${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
#)
#
#target_link_libraries(simple_test PRIVATE
#        "${SOEM_LIB}"
#        pthread
#        rt              # needed on some systems for clock_* APIs
#        # pcap          # only if you built SOEM with the PCAP backend
#)
#
## Add slaveinfo.c to the build
#add_executable(slaveinfo slaveinfo.c)
#
#target_include_directories(slaveinfo PRIVATE
#        "${RAISIN_ROOT}/src/raisin/soem/soem"
#        "${RAISIN_ROOT}/src/raisin/soem/osal"
#        "${RAISIN_ROOT}/src/raisin/soem/osal/linux"
#        "${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
#)
#
#target_link_libraries(slaveinfo PRIVATE
#        "${SOEM_LIB}"
#        pthread
#        rt
#)
#
#
## Add simple_test_XMC.c to the build
#add_executable(simple_test_XMC simple_test_XMC.c)
#
#target_include_directories(simple_test_XMC PRIVATE
#        "${RAISIN_ROOT}/src/raisin/soem/soem"
#        "${RAISIN_ROOT}/src/raisin/soem/osal"
#        "${RAISIN_ROOT}/src/raisin/soem/osal/linux"
#        "${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
#)
#
#target_link_libraries(simple_test_XMC PRIVATE
#        "${SOEM_LIB}"
#        pthread
#        rt
#)
#
#
#
#
#
## Add simple_test_dongdrive.c to the build
#add_executable(simple_test_dongdrive simple_test_dongdrive.c)
#
#target_include_directories(simple_test_dongdrive PRIVATE
#        "${RAISIN_ROOT}/src/raisin/soem/soem"
#        "${RAISIN_ROOT}/src/raisin/soem/osal"
#        "${RAISIN_ROOT}/src/raisin/soem/osal/linux"
#        "${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
#)
#
#target_link_libraries(simple_test_dongdrive PRIVATE
#        "${SOEM_LIB}"
#        pthread
#        rt
#)


cmake_minimum_required(VERSION 3.16)
project(soem_example C)
set(CMAKE_C_STANDARD 11)

# Find the installed SOEM package (lowercase)
find_package(soem CONFIG REQUIRED) # looks for soemConfig.cmake

# Common link libs (raw-socket backend on Linux)
set(COMMON_LINK_LIBS soem pthread rt)
# If you built SOEM with pcap backend, also:
# list(APPEND COMMON_LINK_LIBS pcap)

add_executable(simple_test            src/simple_test.c)
target_link_libraries(simple_test     PRIVATE ${COMMON_LINK_LIBS})

add_executable(slaveinfo              src/slaveinfo.c)
target_link_libraries(slaveinfo       PRIVATE ${COMMON_LINK_LIBS})

add_executable(simple_test_XMC        src/simple_test_XMC.c)
target_link_libraries(simple_test_XMC PRIVATE ${COMMON_LINK_LIBS})

add_executable(simple_test_dongdrive  src/simple_test_dongdrive.c)
target_link_libraries(simple_test_dongdrive PRIVATE ${COMMON_LINK_LIBS})