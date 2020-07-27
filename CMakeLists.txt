cmake_minimum_required(VERSION 3.0.2)
project(gazebo_tf_pub)

find_package(catkin REQUIRED COMPONENTS  roscpp std_msgs std_srvs message_generation) 
# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


###########
## Build ##
###########
# Specify additional locations of header files.
include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_service_files(DIRECTORY srv FILES LinkRequested.srv)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package()

# Build our plugin
add_library(gazebo_tf_plugin SHARED src/gazebo_tf_plugin.cpp)
target_link_libraries(gazebo_tf_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


