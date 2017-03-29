cmake_minimum_required(VERSION 2.8.3)
project(bebop_track_tag)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	std_msgs
)

catkin_package()

include_directories(
	include ${catkin_INCLUDE_DIRS}
)

add_executable( track_tag src/track_tag.cpp )

target_link_libraries( track_tag
	${catkin_LIBRARIES}
)
