/// \file tcp_client.hpp
/// \brief Defines the driver_modem::tcp_client_t class.
#ifndef DRIVER_MODEM___TCP_CLIENT_H
#define DRIVER_MODEM___TCP_CLIENT_H

#include <driver_modem_msgs/endpoint.h>
#include <driver_modem_msgs/tcp_client.h>

#include <boost/asio/ip/tcp.hpp>

#include <functional>

namespace driver_modem {

/// \brief A TCP client that attempts to connect to a remote server.
class tcp_client_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new tcp_client_t.
    /// \param io_service The application's io_service instance.
    /// \param id The unique ID of the TCP client.
    /// \param connection_callback The callback for handling a new connection.
    tcp_client_t(boost::asio::io_service& io_service, uint32_t id, std::function<void(boost::asio::ip::tcp::socket*)> connection_callback);
    ~tcp_client_t();

    // CONTROL
    /// \brief Starts the TCP client.
    /// \param local_endpoint The local endpoint to start the client on.
    /// \param remote_endpoint The remote endpoint for the client to attempt to connect to.
    /// \returns TRUE if the client started successfully, otherwise FALSE.
    bool start(const driver_modem_msgs::endpoint& local_endpoint, const driver_modem_msgs::endpoint& remote_endpoint);
    /// \brief Stops the TCP client.
    void stop();

    // PROPERTIES
    /// \brief Gets the description of the TCP client.
    /// \returns The description of the TCP client.
    driver_modem_msgs::tcp_client description() const;

private:
    // VARIABLES
    /// \brief The unique ID of the client.
    const uint32_t m_id;

    // ENDPOINTS
    /// \brief Stores the local endpoint of the client.
    driver_modem_msgs::endpoint m_local_endpoint;
    /// \brief Stores the requested endpoint for connection.
    driver_modem_msgs::endpoint m_remote_endpoint;

    // ASIO SOCKET
    /// \brief The new socket for the next connection.
    boost::asio::ip::tcp::socket* m_socket;
    /// \brief Handles a new connection.
    void connect_callback(const boost::system::error_code &error);    

    // CALLBACKS
    /// \brief Stores the external callback for when new connections are made.
    std::function<void(boost::asio::ip::tcp::socket*)> m_callback_connection;
};

}

#endif