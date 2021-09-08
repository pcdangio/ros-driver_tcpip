/// \file endpoint.hpp
/// \brief Defines endpoint conversion functions.
#ifndef DRIVER_MODEM___ENDPOINT_H
#define DRIVER_MODEM___ENDPOINT_H

#include <driver_modem_msgs/endpoint.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>

namespace driver_modem {
namespace endpoint {

// TCP
/// \brief Converts a ROS endpoint to an ASIO TCP endpoint.
/// \param endpoint_ros The ROS endpoint to convert.
/// \returns The converted ASIO TCP endpoint.
boost::asio::ip::tcp::endpoint to_asio_tcp(const driver_modem_msgs::endpoint& endpoint_ros);
/// \brief Converts an ASIO TCP endpoint to a ROS endpoint.
/// \param endpoint_asio The ASIO TCP endpoint to convert.
/// \returns The converted ROS endpoint.
driver_modem_msgs::endpoint to_ros(const boost::asio::ip::tcp::endpoint& endpoint_asio);

// UDP
/// \brief Converts a ROS endpoint to an ASIO UDP endpoint.
/// \param endpoint_ros The ROS endpoint to convert.
/// \returns The converted ASIO UDP endpoint.
boost::asio::ip::udp::endpoint to_asio_udp(const driver_modem_msgs::endpoint& endpoint_ros);
/// \brief Converts an ASIO UDP endpoint to a ROS endpoint.
/// \param endpoint_asio The ASIO UDP endpoint to convert.
/// \returns The converted ROS endpoint.
driver_modem_msgs::endpoint to_ros(const boost::asio::ip::udp::endpoint& endpoint_asio);

}}

#endif