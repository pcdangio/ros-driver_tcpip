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

# Add static library for modem_interface.
add_library(modem_interface src/modem_interface.cpp)
# Add dependency on exported targets for built driver_modem_msgs.
add_dependencies(modem_interface ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(modem_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Add executable for driver_modem_node.
add_executable(${PROJECT_NAME}_node src/main.cpp src/ros_node.cpp src/driver.cpp src/udp_connection.cpp src/tcp_connection.cpp)
# Rename target.
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# Add dependency on exported targets for built driver_modem_msgs.
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
# Link target.
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES})

# Install targets.
install(TARGETS modem_interface ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install library headers.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")