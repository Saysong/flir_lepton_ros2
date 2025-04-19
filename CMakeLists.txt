cmake_minimum_required(VERSION 3.5)
project(flir_lepton)

# C++ standard to C++14 (recommended for ROS2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Letpon3 Library source code
add_subdirectory (grabber_lib)

# target_link_libraries( DisplayImage ${OpenCV_LIBS} )

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  grabber_lib/include
  grabber_lib/Lepton_SDK/include
  grabber_lib/tools/include
  /opt/ros/jazzy/include
)

## Declare a C++ library
# add_library(leptonSDKEmb32PUB
# 	# src/leptonSDKEmb32PUB/LEPTON_SDK.c
# 	# src/leptonSDKEmb32PUB/LEPTON_SYS.c
# 	# src/leptonSDKEmb32PUB/LEPTON_OEM.c
# 	src/leptonSDKEmb32PUB/LEPTON_SDK.c
# 	src/leptonSDKEmb32PUB/LEPTON_SYS.c
# 	src/leptonSDKEmb32PUB/LEPTON_OEM.c
#   src/leptonSDKEmb32PUB/LEPTON_AGC.c
#   src/leptonSDKEmb32PUB/LEPTON_I2C_Protocol.c
#   src/leptonSDKEmb32PUB/LEPTON_I2C_Service.c
#   src/leptonSDKEmb32PUB/crc16fast.c
#   src/leptonSDKEmb32PUB/raspi_I2C.c
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/LeptonThread.cpp
  grabber_lib/src/Lepton3.cpp
  grabber_lib/tools/src/stopwatch.cpp
  grabber_lib/Lepton_SDK/src/crc16fast.c
  grabber_lib/Lepton_SDK/src/LEPTON_AGC.c
  grabber_lib/Lepton_SDK/src/LEPTON_I2C_Protocol.c
  grabber_lib/Lepton_SDK/src/LEPTON_I2C_Service.c
  grabber_lib/Lepton_SDK/src/LEPTON_OEM.c
  grabber_lib/Lepton_SDK/src/LEPTON_RAD.c
  grabber_lib/Lepton_SDK/src/LEPTON_SDK.c
  grabber_lib/Lepton_SDK/src/LEPTON_SYS.c
  grabber_lib/Lepton_SDK/src/LEPTON_VID.c
  grabber_lib/Lepton_SDK/src/linux_I2C.c
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
# 	${OpenCV_LIBS}
# )

#############
## Install ##
#############

# Specify dependencies for the target
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Required by ament build system
ament_package()
