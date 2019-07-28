/// \file tcp_connection.h
/// \brief Defines the tcp_connection class.
#ifndef TCP_CONNECTION_H
#define TCP_CONNECTION_H

#include "connection_type.h"

#include <boost/asio.hpp>
#include <boost/shared_array.hpp>
#include <functional>

using namespace boost::asio::ip;

///
/// \brief Provides a single asynchronous TCP connection for a specific IP address and port.
///
class tcp_connection
{
public:
    // CONSTRUCTORS
    ///
    /// \brief tcp_connection Creates a new instance for the TCP connection.
    /// \param io_service The global IO Service to run the connection on.
    /// \param local_endpoint The local endpoint to bind to.
    /// \param remote_endpoint The remote endpoint to communicate with.
    /// \param buffer_size The size of the RX buffer in bytes.
    ///
    tcp_connection(boost::asio::io_service& io_service, tcp::endpoint local_endpoint, tcp::endpoint remote_endpoint, uint32_t buffer_size=1024);
    ~tcp_connection();

    // METHODS
    ///
    /// \brief connect Attempts to create the TCP connection.
    /// \return TRUE if the connection was successful, otherwise false.
    ///
    bool connect();
    ///
    /// \brief attach_rx_callback Attaches a callback for handling received messages.
    /// \param callback The callback to handle received messages.
    ///
    void attach_rx_callback(std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> callback);
    ///
    /// \brief attach_disconnect_callback Attaches a callback for handling when the connected is lost or closed.
    /// \param callback The callback to handle a closed/lost connection.
    ///
    void attach_disconnect_callback(std::function<void()> callback);
    ///
    /// \brief tx Transmits data to the remote endpoint.
    /// \param data The data to transmit.
    /// \param length The length of the data in bytes.
    /// \return TRUE if the data was transmitted, otherwise FALSE.
    ///
    bool tx(uint8_t* data, uint32_t length);

    // PROPERTIES
    ///
    /// \brief p_connected Gets if the socket is connected.
    /// \return TRUE if connected, otherwise FALSE.
    ///
    bool p_connected();

private:
    // VARIABLES
    ///
    /// \brief m_socket The socket implementing the TCP connection.
    ///
    tcp::socket m_socket;
    ///
    /// \brief m_remote_endpoint The remote endpoint to communicate with.
    ///
    tcp::endpoint m_remote_endpoint;
    ///
    /// \brief m_buffer The internal buffer for storing received messages.
    ///
    uint8_t* m_buffer;
    ///
    /// \brief m_buffer_size The size of the internal buffer in bytes.
    ///
    uint32_t m_buffer_size;
    ///
    /// \brief m_connected Internal flag storing if socket is connected.
    ///
    bool m_connected;
    ///
    /// \brief m_rx_callback The callback to raise when a message is received.
    ///
    std::function<void(connection_type, uint16_t, uint8_t*, uint32_t)> m_rx_callback;
    ///
    /// \brief m_disconnect_callback The callback to raise when the connection is closed.
    ///
    std::function<void()> m_disconnect_callback;

    // METHODS
    ///
    /// \brief async_rx Initiates an asynchronous read of a single TCP packet.
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

#endif // TCP_CONNECTION_H
