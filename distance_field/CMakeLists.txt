cmake_minimum_required(VERSION 2.8.3)
project(distance_field)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs)

find_package(Bullet REQUIRED)
include_directories(include ${BULLET_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
	CATKIN_DEPENDS roscpp visualization_msgs
	DEPENDS bullet
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

add_library(distance_field
	src/pf_distance_field.cpp
	src/propagation_distance_field.cpp
)
target_link_libraries(${PROJECT_NAME} ${BULLET_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
target_link_libraries(test_voxel_grid ${PROJECT_NAME})

catkin_add_gtest(test_distance_field test/test_distance_field.cpp)
target_link_libraries(test_distance_field ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

