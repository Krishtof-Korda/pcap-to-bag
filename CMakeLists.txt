cmake_minimum_required(VERSION 3.1.0)
project(pcap_to_bag)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  ouster_ros
  rosbag
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        ouster_ros
)


# Need to wait for libtins version 4.1+
# find_package(libtins REQUIRED)

# and for now just pkg_config check
find_package(PkgConfig REQUIRED)
pkg_check_modules(libtins REQUIRED)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-mavx2 -Wall -Wextra -Werror)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/tins
)

add_executable(
    pcap_to_bag
    src/pcap_to_bag.cpp
)

target_link_libraries(pcap_to_bag ${catkin_LIBRARIES} tins)

add_dependencies(pcap_to_bag ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS pcap_to_bag
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


