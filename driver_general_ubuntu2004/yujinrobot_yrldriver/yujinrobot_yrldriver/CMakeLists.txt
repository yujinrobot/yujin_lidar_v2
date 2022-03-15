##############################################################################
# Cmake
##############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(yujinrobot_yrldriver)
message(STATUS "CMAKEing ${PROJECT_NAME} --")

##############################################################################
# Find Packages
##############################################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLATFORM_CXX_FLAGS} -std=c++11")

if( PLAIN_LINUX_BOX )
else()
endif()

set(SHIELD_TOOLS_DRIVER_TEST_TARGET_LINK     
  pthread
  rt
  dl
)  


##############################################################################
# Includes
##############################################################################
include_directories(include)
##############################################################################
# Actual project configuration
##############################################################################
add_subdirectory(include)
add_subdirectory(src)
