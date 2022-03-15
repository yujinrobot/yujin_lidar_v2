##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(shield_sophus)

message(STATUS "CMAKEing sophus --")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

message(STATUS, "include directories are ${include}") 
#
##############################################################################
# Actual project configuration
##############################################################################
message(STATUS "We are using Sophus-1.0.0")

# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#   message(STATUS "kisanman dir='${dir}'")
# endforeach()

include_directories(include
  "../eigen"
"../eigen/eigen-3.3.7"
)

# install(DIRECTORY Sophus-1.0.0/sophus DESTINATION ${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION})
configure_file( Sophus-1.0.0/sophus/common.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)
configure_file( Sophus-1.0.0/sophus/rxso3.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)
configure_file( Sophus-1.0.0/sophus/se3.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)
configure_file( Sophus-1.0.0/sophus/se2.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)
configure_file( Sophus-1.0.0/sophus/so3.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)
configure_file( Sophus-1.0.0/sophus/so2.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)
configure_file( Sophus-1.0.0/sophus/sim3.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)
configure_file( Sophus-1.0.0/sophus/types.hpp ${CMAKE_CURRENT_SOURCE_DIR}/${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)


if( NOT BUILD_YJFW_FOR_RELEASE )
  add_subdirectory(include)
endif( NOT BUILD_YJFW_FOR_RELEASE ) 
#add_subdirectory(Sophus-1.0.0)

