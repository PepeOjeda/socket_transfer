cmake_minimum_required(VERSION 3.8)
project(socket_transfer)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


# find dependencies
find_package(ament_cmake REQUIRED)
set(AMENT_DEPS
    rclcpp
)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

# Sockets library
add_subdirectory(third_party/Minimal-Socket)

# Install

install(
  DIRECTORY include/socket_transfer third_party/Minimal-Socket/src/header/MinimalSocket
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(MinimalSocket)
    
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
