catkin_add_gtest(constraintTests constraintTests.cpp)
target_link_libraries(constraintTests ${catkin_LIBRARIES} ${PROJECT_NAME})

catkin_add_gtest(controllerAndSpaceDescriptionTests controllerAndSpaceDescriptionTests.cpp)
target_link_libraries(controllerAndSpaceDescriptionTests ${catkin_LIBRARIES} ${PROJECT_NAME})


#catkin_add_gtest(forceSolverTests forceSolverTests.cpp)
#target_link_libraries(forceSolverTests ${catkin_LIBRARIES} ${PROJECT_NAME})

catkin_add_gtest(qpOasesVelocitySolverTests qpOasesVelocitySolverTests.cpp)
target_link_libraries(qpOasesVelocitySolverTests ${catkin_LIBRARIES} ${PROJECT_NAME})
