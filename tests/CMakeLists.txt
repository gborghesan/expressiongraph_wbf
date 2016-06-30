#file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/tests)



find_package(catkin REQUIRED COMPONENTS  roslib )
add_executable(parser_test_pr2 parser_test_pr2.cpp)
target_link_libraries(parser_test_pr2  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(parser_test_lwr parser_test_lwr.cpp)
target_link_libraries(parser_test_lwr   ${catkin_LIBRARIES} ${PROJECT_NAME})

