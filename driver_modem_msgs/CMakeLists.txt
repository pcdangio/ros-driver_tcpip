cmake_minimum_required(VERSION 2.8.3)
project(driver_modem_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation)

# Add messages.
add_message_files(
  FILES
  data_packet.msg
  active_connections.msg)

# Add services.
add_service_files(
  FILES
  set_remote_host.srv
  get_remote_host.srv
  add_tcp_connection.srv
  add_udp_connection.srv
  remove_connection.srv
  remove_all_connections.srv
  send_tcp.srv)

# Generate messages and services.
generate_messages()

# Create catkin package.
catkin_package(CATKIN_DEPENDS message_runtime)