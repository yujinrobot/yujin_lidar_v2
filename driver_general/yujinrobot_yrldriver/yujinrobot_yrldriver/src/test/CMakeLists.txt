##############################################################################
# OS dependencies
##############################################################################

##############################################################################
# Dependencies for test applications (not mother package)
# Do not move below scripts into the mother library package because the package might not need these libraries
##############################################################################
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake)


##############################################################################
# Binaries
##############################################################################
add_executable(test_yrl_library test_yrl_library.cpp)
target_link_libraries(test_yrl_library yujinrobot_yrldriver)
install(TARGETS test_yrl_library RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})

add_executable(test_IP_change test_IP_change.cpp)
target_link_libraries(test_IP_change yujinrobot_yrldriver)
install(TARGETS test_IP_change RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})

add_executable(test_mode_change test_mode_change.cpp)
target_link_libraries(test_mode_change yujinrobot_yrldriver)
install(TARGETS test_mode_change RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})

add_executable(test_recover_network test_recover_network.cpp)
target_link_libraries(test_recover_network yujinrobot_yrldriver)
install(TARGETS test_recover_network RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})



add_executable(test_fw_set_get_param test_fw_set_get_param.cpp)
target_link_libraries(test_fw_set_get_param yujinrobot_yrldriver)
install(TARGETS test_fw_set_get_param RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})

add_executable(test_yrl_library_devel test_yrl_library_devel.cpp)
target_link_libraries(test_yrl_library_devel yujinrobot_yrldriver)
install(TARGETS test_yrl_library_devel RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})

add_executable(test_fw_set_get test_fw_set_get.cpp)
target_link_libraries(test_fw_set_get yujinrobot_yrldriver)
install(TARGETS test_fw_set_get RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})

add_executable(test_fw_calibration test_fw_calibration.cpp)
target_link_libraries(test_fw_calibration yujinrobot_yrldriver)
install(TARGETS test_fw_calibration RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})

add_executable(test_fw_calibration_mode test_fw_calibration_mode.cpp)
target_link_libraries(test_fw_calibration_mode yujinrobot_yrldriver)
install(TARGETS test_fw_calibration_mode RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})

add_executable(test_thread_pool test_thread_pool.cpp)
target_link_libraries(test_thread_pool yujinrobot_yrldriver)
install(TARGETS test_thread_pool RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}/${PROJECT_NAME})
