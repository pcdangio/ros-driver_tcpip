/// \file udp_socket.hpp
/// \brief Defines the driver_modem::udp_socket_t class.
#ifndef DRIVER_MODEM___UDP_SOCKET_H
#define DRIVER_MODEM___UDP_SOCKET_H

#include "socket.hpp"

#include <ros/ros.h>
#include <driver_modem_msgs/udp_packet.h>
#include <driver_modem_msgs/udp_socket.h>

#include <boost/asio/ip/udp.hpp>

#include <functional>
#include <array>

namespace driver_modem {

/// \brief A UDP socket.
class udp_socket_t
    : public socket_t
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new udp_socket instance.
    /// \param io_service The application's io_service instance.
    /// \param id The unique ID of the UDP socket.
    udp_socket_t(boost::asio::io_service& io_service, uint32_t id);
    ~udp_socket_t();

    // METHODS
    /// \brief Opens the socket.
    /// \param local_endpoint The local endpoint to bind the socket to.
    /// \returns TRUE if the socket was opened successfully, otherwise FALSE.
    bool open(driver_modem_msgs::endpoint& local_endpoint);
    void close() override;

    // PROPERTIES
    /// \brief Gets the description of the UDP socket.
    /// \returns The description of the UDP socket.
    driver_modem_msgs::udp_socket description() const;
    bool is_open() const override;
    
private:
    // ASIO SOCKET
    /// \brief The underlying ASIO socket.
    boost::asio::ip::udp::socket m_socket;
    /// \brief A buffer for storing the last received packet bytes.
    std::array<uint8_t, 1024> m_buffer;
    /// \brief The remote endpoint the last packet was received from.
    boost::asio::ip::udp::endpoint m_remote_endpoint;
    /// \brief Starts an asynchronous read operation.
    void async_rx();
    /// \brief The internal callback for handling messages received asynchronously.
    /// \param error The error code provided by the async read operation.
    /// \param bytes_read The number of bytes ready by the async read operation.
    void rx_callback(const boost::system::error_code& error, std::size_t bytes_read);

    // ROS
    /// \brief The publisher of received messages.
    ros::Publisher m_publisher_rx;
    /// \brief The subscriber for messages to transmit.
    ros::Subscriber m_subscriber_tx;
    /// \brief The callback for the transmit subscriber.
    void subscriber_tx(const driver_modem_msgs::udp_packetConstPtr& message);
};

}

#endif