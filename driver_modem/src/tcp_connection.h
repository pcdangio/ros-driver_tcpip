/// \file tcp_connection.h
/// \brief Defines the tcp_connection class.
#ifndef TCP_CONNECTION_H
#define TCP_CONNECTION_H

#include "driver_modem/protocol.h"
#include "driver_modem/tcp_role.h"

#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <functional>

using namespace boost::asio::ip;
using namespace driver_modem;

/// \brief Provides a single asynchronous TCP connection for a specific IP address and port.
class tcp_connection
        : public boost::enable_shared_from_this<tcp_connection>
{
public:    
    // ENUMERATIONS
    /// \brief Enumerates the statuses that the connection may have.
    enum class status
    {
        DISCONNECTED = 0,   ///< The connection is in a disconnected state
        CONNECTED = 1,      ///< The connection is connected
        PENDING = 2         ///< The connection is pending on a connection to a client/server
    };

    // CONSTRUCTORS
    /// \brief Creates a new instance for the TCP connection.
    /// \param io_service The global IO Service to run the connection on.
    /// \param local_endpoint The local endpoint to bind to.
    /// \param buffer_size The size of the RX buffer in bytes.
    tcp_connection(boost::asio::io_service& io_service, tcp::endpoint local_endpoint, uint32_t buffer_size=1024);
    ~tcp_connection();

    // METHODS: START/STOP
    /// \brief Asynchronously starts the connection as a TCP client.
    /// \param remote_endpoint The remote endpoint to connect to.
    /// \return TRUE if the connection process was started, otherwise FALSE.
    bool start_client(tcp::endpoint remote_endpoint);
    /// \brief Asynchronously starts the connection as a TCP server.
    /// \return TRUE if the listening process was started, otherwise FALSE.
    bool start_server();
    /// \brief Disconnects the connection.
    void disconnect();

    // METHODS: CALLBACK ATTACHMENT
    /// \brief Attaches a callback for handling new connection events.
    /// \param callback The callback to handle new connection events.
    /// \details The callback is raised whenever a new connection is established after start_client() or start_server() is called.
    void attach_connected_callback(std::function<void(uint16_t)> callback);
    /// \brief Attaches a callback for handling disconnection events.
    /// \param callback The callback to handle disconnection events.
    /// \details The callback is raised whenever the connection is lost or disconnected from the other side.
    /// It is NOT raised when the disconnect() method is called.
    void attach_disconnected_callback(std::function<void(uint16_t)> callback);
    /// \brief Attaches a callback for handling received messages.
    /// \param callback The callback to handle received messages.
    void attach_rx_callback(std::function<void(protocol, uint16_t, uint8_t*, uint32_t, address)> callback);

    // METHODS: IO
    /// \brief Transmits data to the remote endpoint.
    /// \param data The data to transmit.
    /// \param length The length of the data in bytes.
    /// \return TRUE if the data was transmitted, otherwise FALSE.
    bool tx(const uint8_t *data, uint32_t length);

    // PROPERTIES
    /// \brief Gets the current role of the connection.
    /// \return The role of the connection.
    tcp_role p_role() const;
    /// \brief Gets the status of the connection.
    /// \return The current status of the connection.
    status p_status() const;
    /// \brief Gets the remote endpoint of the connection.
    /// \return The remote endpoint of the connection.
    tcp::endpoint p_remote_endpoint() const;

private:
    // VARIABLES: SOCKET
    /// \brief The socket implementing the TCP connection.
    tcp::socket m_socket;
    /// \brief A TCP acceptor that listens for and accepts connections.
    tcp::acceptor m_acceptor;
    /// \brief The local endpoint assigned to the connection.
    tcp::endpoint m_local_endpoint;

    // VARIABLES: RX BUFFER
    /// \brief The internal buffer for storing received messages.
    uint8_t* m_buffer;
    /// \brief The size of the internal buffer in bytes.
    uint32_t m_buffer_size;

    // VARIABLES: FLAGS
    /// \brief Stores the current role of the connection.
    tcp_role m_role;
    /// \brief Stores the current status of the connection.
    status m_status;

    // VARIABLES: CALLBACKS
    /// \brief The callback to raise when a new connection event occurs.
    std::function<void(uint16_t)> m_connected_callback;
    /// \brief The callback to raise when a new disconnection event occurs.
    std::function<void(uint16_t)> m_disconnected_callback;
    /// \brief The callback to raise when a message is received.
    std::function<void(protocol, uint16_t, uint8_t*, uint32_t, address)> m_rx_callback;

    // METHODS: SOCKET
    /// \brief Initiates an asynchronous listen/acceptance of new connections in SERVER mode
    void async_accept();
    /// \brief Initiates an asynchronous read of a single TCP packet.
    void async_rx();

    // METHODS:
    /// \brief Updates the status of the connection, raising callbacks as necessary.
    /// \param new_status The new status to assign to the connection.
    /// \param signal Indicates if callbacks should be raised/signaled.
    void update_status(status new_status, bool signal = true);

    // CALLBACKS
    /// \brief The callback for handling newly accepted connections from async_accept().
    /// \param error The error passed back from the async_accept() method.
    void accept_callback(const boost::system::error_code& error);
    /// \brief The callback for handling new connections from asynchronous connect calls.
    /// \param error The error passed back from the async connect method.
    void connect_callback(const boost::system::error_code& error);
    /// \brief The internal callback for handling messages received asynchronously.
    /// \param error The error code provided by the async read operation.
    /// \param bytes_read The number of bytes ready by the async read operation.
    void rx_callback(const boost::system::error_code& error, std::size_t bytes_read);
};

#endif // TCP_CONNECTION_H
