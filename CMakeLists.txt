cmake_minimum_required(VERSION 3.8)
project(image_transfer)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


# find dependencies
find_package(ament_cmake REQUIRED)
set(AMENT_DEPS
    rclcpp
    sensor_msgs
)
foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Sockets library
add_subdirectory(third_party/Minimal-Socket)

# Client node
add_executable(client_udp src/client_udp.cpp)
ament_target_dependencies(client_udp ${AMENT_DEPS})
target_link_libraries(client_udp MinimalSocket)

# server node
add_executable(server_udp src/server_udp.cpp)
ament_target_dependencies(server_udp ${AMENT_DEPS})
target_link_libraries(server_udp MinimalSocket)


# Install
install(
  TARGETS client_udp server_udp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
