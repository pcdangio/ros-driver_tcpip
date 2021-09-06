/// \file tcp_server.hpp
/// \brief Defines the driver_modem::tcp_server_t class.
#ifndef DRIVER_MODEM___TCP_SERVER_H
#define DRIVER_MODEM___TCP_SERVER_H

#include <boost/asio/ip/tcp.hpp>

namespace driver_modem {

/// \brief A TCP server for accepting incoming connections.
class tcp_server_t
{
public:
    // CONSTRUCOTRS
    /// \brief Instantiates a new tcp_server_t.
    /// \param io_service The application's io_service instance.
    /// \param id The unique ID of the TCP server.
    tcp_server_t(boost::asio::io_service& io_service, uint32_t id);
    ~tcp_server_t();

private:
    /// \brief The TCP acceptor for the server.
    boost::asio::ip::tcp::acceptor m_acceptor;
    /// \brief The unique ID of the server.
    const uint32_t m_id;
};

}

#endif