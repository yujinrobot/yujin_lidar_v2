###############################################################################
# FILES
###############################################################################

FILE(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

INSTALL(FILES ${HEADERS} DESTINATION ${SHIELD_TOOLS_GLOBAL_INCLUDE_DESTINATION}/shield_sophus)
