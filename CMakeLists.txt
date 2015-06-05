cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_library(matlab_reporter
  src/utils/matlab_reporter.cpp)
rosbuild_add_library(controllers
  src/controllers/position_controller.cpp
  src/controllers/rotation_controller.cpp
  src/controllers/position_ff_controller.cpp  )  
rosbuild_add_library(space_description
  src/solver/space_description.cpp)  
rosbuild_add_library(simple_force_solver
  src/solver/simple_force_solver.cpp) 
   
target_link_libraries(matlab_reporter expressiongraph)
target_link_libraries(controllers expressiongraph)
target_link_libraries(space_description expressiongraph)
target_link_libraries(simple_force_solver expressiongraph controllers space_description)




ADD_SUBDIRECTORY(tests)