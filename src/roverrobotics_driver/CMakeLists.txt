cmake_minimum_required(VERSION 3.8)
project(roverrobotics_driver)

# Default to C++17
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-g)
endif()

# <-- Add this to enable std::filesystem
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(rover_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(diagnostic_updater REQUIRED)

############### development ###############

find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(greenhouse_experiment_node
  src/greenhouse_experiment_node.cpp
)

ament_target_dependencies(greenhouse_experiment_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs

  # tf2
  # tf2_ros
  # tf2_geometry_msgs

  yaml-cpp
  ament_index_cpp
  Eigen3
)

# Link yaml-cpp library so YAML::LoadFile, Node::as<>() etc. are resolved:
target_link_libraries(greenhouse_experiment_node
  yaml-cpp
  ${ament_index_cpp_LIBRARIES}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

install(TARGETS greenhouse_experiment_node
  DESTINATION lib/${PROJECT_NAME}
)


############### development ###############

#include_directories(include/)
#file(GLOB_RECURSE AllHeaders ${PROJECT_SOURCE_DIR}/*.hpp)

#add_library(librover SHARED IMPORTED)
#set_target_properties(librover PROPERTIES
#  IMPORTED_LOCATION "/usr/lib/liblibrover.so"
#  INTERFACE_INCLUDE_DIRECTORIES "/usr/include/librover")




add_executable(roverrobotics_driver
  src/roverrobotics_ros2_driver.cpp
  library/librover/src/protocol_pro.cpp
  library/librover/src/comm_serial.cpp
  library/librover/src/utils.cpp
  library/librover/src/comm_can.cpp
  library/librover/src/control.cpp
  library/librover/src/vesc.cpp
  library/librover/src/utilities.cpp
  library/librover/src/protocol_zero_2.cpp
  library/librover/src/differential_robot.cpp)

#target_link_libraries(roverrobotics_driver librover)



target_include_directories(roverrobotics_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/library/librover/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(roverrobotics_driver
  rclcpp
  std_msgs
  geometry_msgs
  #rover_msgs
  nav_msgs
  tf2_geometry_msgs
  sensor_msgs
  #diagnostic_updater
  )

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  roverrobotics_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
