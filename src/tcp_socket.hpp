/// \file tcp_socket.hpp
/// \brief Defines the driver_modem::tcp_socket_t class.
#ifndef DRIVER_MODEM___TCP_SOCKET_H
#define DRIVER_MODEM___TCP_SOCKET_H

#include <ros/ros.h>
#include <driver_modem_msgs/endpoint.h>

#include <boost/asio/ip/tcp.hpp>

#include <memory>

namespace driver_modem {

/// \brief A TCP socket.
class tcp_socket_t
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new tcp_socket instance with an existing ASIO socket.
    /// \param socket The existing ASIO socket to instantiate with.
    /// \param id The unique ID of the TCP socket.
    tcp_socket_t(boost::asio::ip::tcp::socket* socket, uint32_t id);
    /// \brief Creates a new tcp_socket instance with a new ASIO socket.
    /// \param io_service The application's io_service instance.
    /// \param id The unique ID of the TCP socket.
    tcp_socket_t(boost::asio::io_service& io_service, uint32_t id);
    ~tcp_socket_t();

    /// \brief Connects to an existing TCP server.
    /// \param local_endpoint The local endpoint of the socket.
    /// \param remote_endpoint The remote endpoint to connect to.
    /// \returns TRUE if the connection succeeded, otherwise FALSE.
    bool connect(driver_modem_msgs::endpoint& local_endpoint, driver_modem_msgs::endpoint& remote_endpoint);
    /// \brief Closes the socket.
    void close();

private:
    // SOCKET
    /// \brief The underlying ASIO socket.
    boost::asio::ip::tcp::socket* m_socket;
    /// \brief The unique ID of the socket.
    const uint32_t m_id;

    // ENDPOINT CONVERSION
    /// \brief Converts a ROS endpoint to an ASIO endpoint.
    /// \param endpoint_ros The ROS endpoint to convert.
    /// \returns The converted ASIO endpoint.
    boost::asio::ip::tcp::endpoint endpoint_asio(const driver_modem_msgs::endpoint& endpoint_ros) const;
    /// \brief Converts an ASIO endpoint to a ROS endpoint.
    /// \param endpoint_asio The ASIO endpoint to convert.
    /// \returns The converted ROS endpoint.
    driver_modem_msgs::endpoint endpoint_ros(const boost::asio::ip::tcp::endpoint& endpoint_asio) const;
};

}

#endif