##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Library
##############################################################################
add_library(yujinrobot_yrldriver SHARED ${SOURCES})
target_link_libraries(yujinrobot_yrldriver ${SHIELD_TOOLS_DRIVER_TEST_TARGET_LINK})

##############################################################################
# Installs
##############################################################################

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${SHIELD_TOOLS_GLOBAL_BIN_DESTINATION}
    ARCHIVE DESTINATION ${SHIELD_TOOLS_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${SHIELD_TOOLS_PACKAGE_LIB_DESTINATION}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
       
 
 
 
