cmake_minimum_required (VERSION 2.8.3)

set (CMAKE_CXX_STANDARD 11)
set (Boost_USE_STATIC_LIBS OFF)
set (Boost_USE_MULTITHREADED ON)
set (Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

include_directories(${Cedes_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})
add_library(cedes tcp_connection.cpp udp_server.cpp)
target_link_libraries(cedes ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
