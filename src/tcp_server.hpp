/// \file tcp_server.hpp
/// \brief Defines the driver_modem::tcp_server_t class.
#ifndef DRIVER_MODEM___TCP_SERVER_H
#define DRIVER_MODEM___TCP_SERVER_H

#include <driver_modem_msgs/endpoint.h>

#include <boost/asio/ip/tcp.hpp>

#include <functional>

namespace driver_modem {

/// \brief A TCP server for accepting incoming connections.
class tcp_server_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new tcp_server_t.
    /// \param io_service The application's io_service instance.
    /// \param id The unique ID of the TCP server.
    tcp_server_t(boost::asio::io_service& io_service, uint32_t id, std::function<void(uint32_t, boost::asio::ip::tcp::socket*)> connection_callback);
    ~tcp_server_t();

    // CONTROL
    /// \brief Starts the TCP server.
    /// \param local_endpoint The local endpoint to start the server on.
    /// \returns TRUE if the server started successfully, otherwise FALSE.
    bool start(driver_modem_msgs::endpoint& local_endpoint);
    /// \brief Stops the TCP server.
    void stop();

private:
    // VARIABLES
    /// \brief The unique ID of the server.
    const uint32_t m_id;

    // ASIO ACCEPTOR
    /// \brief The TCP acceptor for the server.
    boost::asio::ip::tcp::acceptor m_acceptor;
    /// \brief The new socket for the next connection.
    boost::asio::ip::tcp::socket* m_socket;
    /// \brief Starts asynchronously accepting connections.
    void async_accept();
    /// \brief Handles the acceptance of new connections.
    void accept_callback(const boost::system::error_code &error);    

    // CALLBACKS
    /// \brief Stores the external callback for when new connections are made.
    std::function<void(uint32_t, boost::asio::ip::tcp::socket*)> m_callback_connection;
};

}

#endif