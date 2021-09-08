cmake_minimum_required(VERSION 2.8.3)
project(driver_tcpip)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  driver_tcpip_msgs)

# Set up catkin package.
catkin_package(
  CATKIN_DEPENDS roscpp driver_tcpip_msgs
)

# Set up include directories.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add executable for driver_tcpip_node.
add_executable(${PROJECT_NAME}_node src/socket.cpp src/udp_socket.cpp src/tcp_socket.cpp src/tcp_server.cpp src/tcp_client.cpp src/endpoint.cpp src/driver_tcpip.cpp src/main.cpp)
# Rename target.
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME driver_tcpip PREFIX "")
# Link target.
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES})

# Install targets.
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})