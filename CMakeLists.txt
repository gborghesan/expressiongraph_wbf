cmake_minimum_required(VERSION 2.8.3)
project(expressiongraph_wbf)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS expressiongraph urdf)
find_package(rostest REQUIRED)
include_directories( include ${catkin_INCLUDE_DIRS})



# external libraries:

execute_process(COMMAND svn co https://projects.coin-or.org/svn/qpOASES/stable/3.2 ${PROJECT_SOURCE_DIR}/external/qpOASES)


SET(qpOASES qpOASES)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/external/${qpOASES}/include)
add_definitions( -D__NO_COPYRIGHT__ )
ADD_SUBDIRECTORY( external )

add_library(${PROJECT_NAME}
  src/controllers/scalar_controllers.cpp
  src/controllers/rotation_controllers.cpp
  src/solver/constraints.cpp
  src/solver/space_description.cpp
  src/solver/simple_force_solver.cpp
 # src/solver/velocity_solver.cpp
  src/utils/matlab_reporter.cpp
  src/controllers/scalar_controllers.cpp
  src/controllers/rotation_controllers.cpp
  src/solver/space_description.cpp
  src/utils/Urdf2Expr.cpp)
   


target_link_libraries(${PROJECT_NAME}  qpOASES ${catkin_LIBRARIES})
   

# these lines have the only practical effects to see the include dir in qtcreator Project view

FILE(GLOB_RECURSE LibFiles "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})

catkin_package(
  INCLUDE_DIRS include ${PROJECT_SOURCE_DIR}/external/${qpOASES}/include
  LIBRARIES ${PROJECT_NAME}  expressiongraph
  CATKIN_DEPENDS expressiongraph
  DEPENDS expressiongraph)




# BUILDING AND LINKING TESTS
#catkin_add_gtest(tag test/tag.cpp)
#target_link_libraries(tag ${catkin_LIBRARIES} ${PROJECT_NAME})



ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/tests) 
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/gtests)

#OPTION(ENABLE_EXAMPLES "enable compilation of a series of examples" ON)
install(DIRECTORY external/${qpOASES}/include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)


#install(DIRECTORY examples/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
#  PATTERN ".svn" EXCLUDE)
