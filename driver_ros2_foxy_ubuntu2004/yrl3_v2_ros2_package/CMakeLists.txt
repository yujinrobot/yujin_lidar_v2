cmake_minimum_required(VERSION 3.5)
project(yrl3_v2_ros2_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.3.7)
set(SOPHUS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sophus/include)
set(LIB_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/lib_yujinrobot_yrldriver/lib/libyujinrobot_yrldriver.so)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib_yujinrobot_yrldriver/include/yujinrobot_yrldriver)

include_directories(
    ${EIGEN_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${INCLUDE_DIR}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(yrl3_v2_ros2_node src/yrl3_v2_ros2_node.cpp)
ament_target_dependencies(yrl3_v2_ros2_node rclcpp rcl_interfaces geometry_msgs sensor_msgs tf2_geometry_msgs tf2 tf2_ros)

target_link_libraries(yrl3_v2_ros2_node pthread rt dl ${LIB_SOURCE})

install(TARGETS
  yrl3_v2_ros2_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
