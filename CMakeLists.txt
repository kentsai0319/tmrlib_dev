cmake_minimum_required(VERSION 2.8.3)
project(tmrlib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    tmrdriver
)

###########
## Build ##
###########

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "-std=c++14")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()


## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(tmrdriver
  src/tmrl/driver/driver.cpp
  src/tmrl/driver/tmsct_client.cpp
  src/tmrl/driver/script_commands.cpp
  src/tmrl/driver/sim_pvt_motion.cpp
  src/tmrl/driver/tmsvr_client.cpp
  src/tmrl/driver/robot_state.cpp
  src/tmrl/comm/client.cpp
  src/tmrl/comm/packet.cpp
  src/tmrl/utils/logger.cpp
)
target_link_libraries(tmrdriver
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
# add_dependencies(tmrdriver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(tmrdriver)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(tmrdriver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(tmrdriver
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS tmrdriver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/tmr/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

