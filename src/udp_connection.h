/// \file udp_connection.h
/// \brief Defines the udp_connection class.
#ifndef UDP_CONNECTION_H
#define UDP_CONNECTION_H

#include "connection_type.h"

#include <boost/asio.hpp>
#include <functional>

using namespace boost::asio::ip;

///
/// \brief Provides a single asynchronous UDP connection for a specific IP address and port.
///
class udp_connection
{
public:
    // CONSTRUCTORS
    ///
    /// \brief udp_connection Creates a new instance for the UDP connection.
    /// \param io_service The global IO Service to run the connection on.
    /// \param local_endpoint The local endpoint to bind to.
    /// \param buffer_size The size of the RX buffer in bytes.
    ///
    udp_connection(boost::asio::io_service& io_service, udp::endpoint local_endpoint, uint32_t buffer_size=1024);
    ~udp_connection();

    // METHODS
    ///
    /// \brief attach_rx_callback Attaches a callback for handling received messages.
    /// \param callback The callback to handle receieved messages.
    ///
    void attach_rx_callback(std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> callback);
    ///
    /// \brief tx Transmits data to the remote endpoint.
    /// \param data The data to transmit.
    /// \param length The length of the data in bytes.
    ///
    void tx(const uint8_t *data, uint32_t length);

private:
    // VARIABLES: SOCKET
    ///
    /// \brief m_socket The socket implementing the UDP connection.
    ///
    udp::socket m_socket;
    ///
    /// \brief m_remote_endpoint Stores the remote endpoint information of received messages.
    ///
    udp::endpoint m_remote_endpoint;

    // VARIABLES: RX BUFFER
    ///
    /// \brief m_buffer The internal buffer for storing received messages.
    ///
    uint8_t* m_buffer;
    ///
    /// \brief m_buffer_size The size of the internal buffer in bytes.
    ///
    uint32_t m_buffer_size;

    // VARIABLES: CALLBACKS
    ///
    /// \brief m_rx_callback The callback to raise when a message is received.
    ///
    std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> m_rx_callback;

    // METHODS
    ///
    /// \brief async_rx Initiates an asynchronous read of a single UDP packet.
    ///
    void async_rx();

    // CALLBACKS
    ///
    /// \brief rx_callback The internal callback for handling messages received asynchronously.
    /// \param error The error code provided by the async read operation.
    /// \param bytes_read The number of bytes ready by the async read operation.
    ///
    void rx_callback(const boost::system::error_code& error, std::size_t bytes_read);
};

#endif // UDP_CONNECTION_H
