add_definitions(-DENABLE_CV_HELPERS=1)
#SET (ON_DESKTOP FALSE CACHE BOOL "let them know this is for pc")
add_definitions(-DARM_ONBOARD_BUILDING=1)
SET (ARM_ONBOARD_BUILDING TRUE)

# Flags
# from : https://www.funtoo.org/ODROID-XU4 and -mcpu is deprecated
set(PLATFORM_CXX_FLAGS "-DENABLE_NEON -march=armv7-a -mtune=cortex-a15.cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -Ofast -pipe")

# add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)
# add_definitions(-DEIGEN_DONT_VECTORIZE)
# add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

MESSAGE(STATUS "On board Compiling - ARM")
