cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME usv_ign_platform)
set(NODE_NAME usv_ign_platform)
project(${PROJECT_NAME} VERSION 0.1.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  Eigen3
  as2_core
  ros_ign_bridge
  ros_ign_gazebo
  tf2_geometry_msgs
  image_transport
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Citadel and foxy
if("$ENV{IGNITION_VERSION}" STREQUAL "citadel" AND "$ENV{ROS_DISTRO}" STREQUAL "foxy")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})

  find_package(ignition-transport8 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Citadel")
  add_definitions(-DIGNITION_CITADEL)
# Fortress and galactic or Fortress and foxy
else()
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
  add_definitions(-DIGNITION_FORTRESS)
endif()

set(SOURCE_CPP_FILES 
  lib/ignition_bridge.cpp
  src/${NODE_NAME}.cpp
)
set(HEADER_HPP_FILES
  include/${NODE_NAME}/${NODE_NAME}.hpp
  include/${NODE_NAME}/ignition_bridge.hpp
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${NODE_NAME}_main.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  sensor_msgs 
  std_msgs
  std_srvs
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  image_transport
  ros_ign_bridge
  ros_ign_gazebo
)

if("$ENV{IGNITION_VERSION}" STREQUAL "citadel" AND "$ENV{ROS_DISTRO}" STREQUAL "foxy")
  ament_target_dependencies(${PROJECT_NAME}_node 
    ignition-transport8
    ignition-msgs5
  )

else()
  ament_target_dependencies(${PROJECT_NAME}_node 
    ignition-transport11
    ignition-msgs8
  )

endif()

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
