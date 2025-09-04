cmake_minimum_required(VERSION 3.30)
project(soem_example)

set(CMAKE_CXX_STANDARD 20)
set(RAISIN_ROOT "$ENV{HOME}/Desktop/raisin_ws/raisin_master")
set(RAISIN_BUILD "$ENV{HOME}/Desktop/raisin_ws/raisin_master/build")
set(SOEM_LIB "${RAISIN_BUILD}/src/raisin/soem/libsoem.a")

add_library(soem STATIC IMPORTED GLOBAL)
set_target_properties(soem PROPERTIES
        IMPORTED_LOCATION "${SOEM_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES
        "${RAISIN_ROOT}/src/raisin/soem/soem;${RAISIN_ROOT}/src/raisin/soem/osal;${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
)

# Add path to SOEM's cmake config
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Desktop/SOEM-install/cmake")

## Find SOEM package
#find_package(soem REQUIRED)

# Define executable
#add_executable(soem_app main.c)
#
### Link SOEM
##target_link_libraries(soem_app PRIVATE soem)
## Link your app
#target_link_libraries(soem_app PRIVATE
#        soem
#        pthread           # SOEM on Linux typically needs pthread
#        # rt              # uncomment if you get unresolved clock_* / timer_* symbols
#        # pcap            # only if you use the PCAP NIC backend
#)

add_executable(simple_test simple_test.c)

# SOEM headers inside raisin (so ethercat.h resolves)
# SOEM headers (add the linux OSAL folder!)
target_include_directories(simple_test PRIVATE
        "${RAISIN_ROOT}/src/raisin/soem/soem"        # ethercat.h, etc.
        "${RAISIN_ROOT}/src/raisin/soem/osal"
        "${RAISIN_ROOT}/src/raisin/soem/osal/linux"  # <— THIS fixes osal_defs.h
        "${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
)

target_link_libraries(simple_test PRIVATE
        "${SOEM_LIB}"
        pthread
        rt              # needed on some systems for clock_* APIs
        # pcap          # only if you built SOEM with the PCAP backend
)

# Add slaveinfo.c to the build
add_executable(slaveinfo slaveinfo.c)

target_include_directories(slaveinfo PRIVATE
        "${RAISIN_ROOT}/src/raisin/soem/soem"
        "${RAISIN_ROOT}/src/raisin/soem/osal"
        "${RAISIN_ROOT}/src/raisin/soem/osal/linux"
        "${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
)

target_link_libraries(slaveinfo PRIVATE
        "${SOEM_LIB}"
        pthread
        rt
)


# Add simple_test_XMC.c to the build
add_executable(simple_test_XMC simple_test_XMC.c)

target_include_directories(simple_test_XMC PRIVATE
        "${RAISIN_ROOT}/src/raisin/soem/soem"
        "${RAISIN_ROOT}/src/raisin/soem/osal"
        "${RAISIN_ROOT}/src/raisin/soem/osal/linux"
        "${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
)

target_link_libraries(simple_test_XMC PRIVATE
        "${SOEM_LIB}"
        pthread
        rt
)





# Add simple_test_dongdrive.c to the build
add_executable(simple_test_dongdrive simple_test_dongdrive.c)

target_include_directories(simple_test_dongdrive PRIVATE
        "${RAISIN_ROOT}/src/raisin/soem/soem"
        "${RAISIN_ROOT}/src/raisin/soem/osal"
        "${RAISIN_ROOT}/src/raisin/soem/osal/linux"
        "${RAISIN_ROOT}/src/raisin/soem/oshw/linux"
)

target_link_libraries(simple_test_dongdrive PRIVATE
        "${SOEM_LIB}"
        pthread
        rt
)