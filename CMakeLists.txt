cmake_minimum_required(VERSION 2.6)
enable_testing()
project(gtsamexamples CXX C)

# option: whether turn on Matlab toolbox
option(EXAMPLES_BUILD_MATLAB_TOOLBOX "whether build matlab toolbox" OFF)


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})


# Process source subdirs
add_subdirectory(cpp)


# Wrapping to MATLAB
if(EXAMPLES_BUILD_MATLAB_TOOLBOX)
  # wrap
  include(GtsamMatlabWrap)
  wrap_and_install_library(gtsamexamples.h ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}" "")
endif()

# Install config and export files
#GtsamMakeConfigFile(gtsamexamples)
#export(TARGETS ${gtsamexamples_EXPORTED_TARGETS} FILE gtsamexamples-exports.cmake)

