# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(ur5_notebook)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp geometry_msgs std_srvs std_msgs message_generation)
# Declare our catkin package.

#add_message files
add_message_files(FILES blocks_poses.msg)
add_message_files(FILES Tracker.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(blocks_spawner blocks_spawner.cpp)
add_executable(blocks_poses_publisher blocks_poses_publisher.cpp)

# Specify libraries against which to link.
target_link_libraries(blocks_spawner ${catkin_LIBRARIES})
target_link_libraries(blocks_poses_publisher ${catkin_LIBRARIES})
