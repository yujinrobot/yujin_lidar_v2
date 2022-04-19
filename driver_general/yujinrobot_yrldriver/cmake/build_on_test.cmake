add_definitions(-DENABLE_CV_HELPERS=1)
#SET (ON_DESKTOP FALSE CACHE BOOL "let them know this is for pc")
add_definitions(-DARM_ONBOARD_BUILDING=1)
SET (ARM_ONBOARD_BUILDING TRUE)

add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)
add_definitions(-DEIGEN_DONT_VECTORIZE)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

MESSAGE(STATUS "On board Compiling - TEST")
