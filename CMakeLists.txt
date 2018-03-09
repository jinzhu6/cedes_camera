cmake_minimum_required(VERSION 2.8.3)
project(cedes)

set (CMAKE_CXX_STANDARD 11)
set (Boost_USE_STATIC_LIBS OFF)
set (Boost_USE_MULTITHREADED ON)
set (Boost_USE_STATIC_RUNTIME OFF)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rosgraph_msgs rospy std_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads)

add_message_files(
  FILES
  NodeExampleData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/cedes.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs rospy std_msgs
)

include_directories(${cedes_SOURCE_DIR}/include/${PROJECT_NAME} ${Boost_INCLUDE_DIRS})
add_library(driver src/tcp_connection.cpp src/udp_server.cpp src/interface.cpp)
target_link_libraries(driver ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}) 

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cedes src/nodes/cedes_node.cpp)
target_link_libraries(cedes ${catkin_LIBRARIES} driver)
add_dependencies(cedes cedes_gencfg cedes_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS cedes
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
