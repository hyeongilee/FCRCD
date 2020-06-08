cmake_minimum_required(VERSION 2.8.3)
project(my_service)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


add_service_files(
    DIRECTORY srv
    FILES location.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES service
#  CATKIN_DEPENDS message_generation std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
