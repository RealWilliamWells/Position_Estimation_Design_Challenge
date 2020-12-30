cmake_minimum_required(VERSION 3.0.2)
project(gps_simulator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation dynamics_simulator)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
        ${catkin_INCLUDE_DIRS}
        )

## Declare a cpp executables
add_executable(${PROJECT_NAME} src/gps_simulator_node.cpp)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
add_dependencies(dynamics_simulator ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        )