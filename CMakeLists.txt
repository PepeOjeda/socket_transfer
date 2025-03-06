cmake_minimum_required(VERSION 3.8)
project(socket_transfer)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fmt REQUIRED)


# Sockets library
add_subdirectory(third_party/Minimal-Socket)

include_directories(include)

# HACK: empty library to get proper IDE support with ament dependencies
add_library(sockets src/sockets.cpp)
ament_target_dependencies(sockets rclcpp rclcpp_action std_msgs)
target_link_libraries(sockets fmt MinimalSocket)

# Install
install(
  DIRECTORY include/socket_transfer third_party/Minimal-Socket/src/header/MinimalSocket
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(MinimalSocket fmt)
ament_export_dependencies(rclcpp rclcpp_action std_msgs)
    
install(TARGETS MinimalSocket
  DESTINATION lib
)


ament_package()
