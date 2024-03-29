/// \file tcp_socket.hpp
/// \brief Defines the driver_tcpip::tcp_socket_t class.
#ifndef DRIVER_TCPIP___TCP_SOCKET_H
#define DRIVER_TCPIP___TCP_SOCKET_H

#include "socket.hpp"

#include <ros/ros.h>
#include <driver_tcpip_msgs/endpoint.h>
#include <driver_tcpip_msgs/send_tcp.h>
#include <driver_tcpip_msgs/tcp_socket.h>

#include <boost/asio/ip/tcp.hpp>

#include <memory>

namespace driver_tcpip {

/// \brief A TCP socket.
class tcp_socket_t
    : public socket_t
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new tcp_socket instance from an existing ASIO socket.
    /// \param socket The existing ASIO socket to instantiate with.
    /// \param id The unique ID of the TCP socket.
    tcp_socket_t(boost::asio::ip::tcp::socket* socket, uint32_t id);
    ~tcp_socket_t();

    void close() override;

    // PROPERTIES
    /// \brief Gets the description of the TCP socket.
    /// \returns The description of the TCP socket.
    driver_tcpip_msgs::tcp_socket description() const;
    bool is_open() const override;

private:
    // ASIO SOCKET
    /// \brief The underlying ASIO socket.
    boost::asio::ip::tcp::socket* m_socket;
    /// \brief A buffer for storing the last received packet bytes.
    std::array<uint8_t, 1024> m_buffer;
    /// \brief Starts an asynchronous read operation.
    void async_rx();
    /// \brief The internal callback for handling messages received asynchronously.
    /// \param error The error code provided by the async read operation.
    /// \param bytes_read The number of bytes ready by the async read operation.
    void rx_callback(const boost::system::error_code& error, std::size_t bytes_read);

    // ROS
    /// \brief The publisher of received messages.
    ros::Publisher m_publisher_rx;
    /// \brief The service for transmitting messages.
    ros::ServiceServer m_service_tx;
    /// \brief The service callback for transmitting messages.
    /// \param request The transmit service request.
    /// \param response The transmit service response.
    /// \returns TRUE if the service completed, otherwise FALSE.
    bool service_tx(driver_tcpip_msgs::send_tcpRequest& request, driver_tcpip_msgs::send_tcpResponse& response);
    /// \brief Starts all necessary ROS publishers and services.
    void start_ros();
    /// \brief Stops all ROS publishers and services.
    void stop_ros();
};

}

#endif