cmake_minimum_required(VERSION 2.8.3)
project(yujinrobot_yrldriver)
message(STATUS "CMAKEing yrldriver --")
include_directories(include
"../eigen-3.3.7"
"../sophus/include/"
)
  

set (SHIELD_TOOLS_GLOBAL_BIN_DESTINATION "bin")
set (SHIELD_TOOLS_PACKAGE_LIB_DESTINATION "lib")
set (SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION "include")
set (SHIELD_TOOLS_GLOBAL_INSTALL_DESTINATION "lib_yujinrobot_yrldriver")

set (CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/${SHIELD_TOOLS_GLOBAL_INSTALL_DESTINATION}")  

add_subdirectory("yujinrobot_yrldriver")
