cmake_minimum_required(VERSION 2.8.3)
project(driver_modem)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  driver_modem_msgs)

# Set up catkin package.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES modem_interface
  CATKIN_DEPENDS roscpp driver_modem_msgs
)

# Set up include directories.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Add static library for modem_interface.
# add_library(modem_interface src/modem_interface.cpp)
# # Add dependency on exported targets for built driver_modem_msgs.
# add_dependencies(modem_interface ${catkin_EXPORTED_TARGETS})

# Add executable for driver_modem_node.
add_executable(${PROJECT_NAME}_node src/socket.cpp src/udp_socket.cpp src/tcp_socket.cpp src/tcp_server.cpp)
# Rename target.
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME driver_modem PREFIX "")
# Add dependency on exported targets for built driver_modem_msgs.
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
# Link target.
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES})

# # Install targets.
# install(TARGETS modem_interface ${PROJECT_NAME}_node
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# # Install library headers.
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h")