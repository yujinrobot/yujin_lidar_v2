cmake_minimum_required(VERSION 3.0.2)
project(yujin_yrl_v2_package)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib_yujinrobot_yrldriver/lib)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib_yujinrobot_yrldriver/include/yujinrobot_yrldriver)
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.3.7)
set(SOPHUS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sophus/include)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf
)

#add_message_files(FILES PointCloud_Msg.msg
#generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
LIBRARIES yujin_yrl_v2_package
CATKIN_DEPENDS std_msgs roscpp sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

add_executable(yrl_pub src/yrl_pub.cpp)
add_dependencies(yrl_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yrl_pub ${catkin_LIBRARIES} pthread rt dl ${LIB_DIR}/libyujinrobot_yrldriver.so)
