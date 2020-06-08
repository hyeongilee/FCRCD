cmake_minimum_required(VERSION 2.8.3)
project(catching_robot)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs cv_bridge image_transport message_filters sensor_msgs my_service geometry_msgs)
find_package(OpenCV REQUIRED)
find_package(k4a 1.2.0 QUIET REQUIRED)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES catching_robot
  CATKIN_DEPENDS std_msgs roscpp sensor_msgs message_filters
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(percept_object src/percept_object.cpp)
add_dependencies(percept_object ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


set(K4A_LIBS k4a::k4a)

target_link_libraries(percept_object ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${K4A_LIBS})

add_executable(move_robot src/move_robot.cpp)
add_dependencies(move_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_robot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${K4A_LIBS})
