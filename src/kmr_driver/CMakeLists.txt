##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(kmr_driver)

##############################################################################
# Catkin
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        ecl_build
        ecl_mobile_robot
        ecl_converters
        ecl_devices
        ecl_geometry
        ecl_sigslots
        ecl_time
        ecl_command_line
)

##############################################################################
# Exports
##############################################################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES kmr
    CATKIN_DEPENDS
        ecl_command_line
        ecl_converters
        ecl_devices
        ecl_geometry
        ecl_mobile_robot
        ecl_sigslots
        ecl_time
)

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
include_directories(include ${catkin_INCLUDE_DIRS})

##############################################################################
# Sources
##############################################################################

add_subdirectory(src)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
